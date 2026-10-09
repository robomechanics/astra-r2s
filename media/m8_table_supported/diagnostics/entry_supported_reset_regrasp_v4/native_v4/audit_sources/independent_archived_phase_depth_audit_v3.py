"""Read-only geometry of original all-step phase/depth; no force simulation."""
from pathlib import Path
import argparse
import hashlib
import json
import xml.etree.ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parents[3]


def wrap_phase(value, pitch):
    value = np.asarray(value, dtype=float)
    return value-pitch*np.floor(value/pitch+.5)


def radial_profile(phase, pitch, pitch_radius, *, female):
    """Independent radial zero-isosurface of the archived ISO-profile kernel."""
    phase = abs(wrap_phase(phase, pitch))
    height = np.sqrt(3.)*pitch/2
    major, minor = pitch_radius+3*height/8, pitch_radius-height/4
    straight = pitch_radius+np.sqrt(3.)*(pitch/4-phase)
    if female:
        return np.clip(straight, minor, major)
    radius = height/6
    root = pitch_radius-height/2+2*radius-np.sqrt(
        np.maximum(radius*radius-(pitch/2-phase)**2, 0.))
    return np.where(phase <= pitch/16, major,
        np.where(phase >= 3*pitch/8, root, straight))


def profile_interference(phase_offset, pitch, male_pitch_radius, female_pitch_radius,
                         *, sample_count=16384):
    """Sample whole circumference; positive means coaxial profile interference.

    This is a geometric lower bound on maximum radial interference, not an
    SDF penetration or a load. Sampling error≤sqrt(3)*pitch/sample_count.
    It excludes tilt, eccentricity, end chamfers, and finite-span availability.
    """
    phase = np.linspace(-pitch/2, pitch/2, sample_count, endpoint=False)
    difference = (radial_profile(phase+phase_offset, pitch, male_pitch_radius, female=False)
        -radial_profile(phase, pitch, female_pitch_radius, female=True))
    return float(difference.max())


def audit(path, parent=None, state_parent=None, yaw_note=None):
    path = Path(path)
    declaration = json.loads((path/'declaration.json').read_text())
    raw = np.load(path/'original_native_force_ledger.npz', allow_pickle=False)
    xmlpath = path/'scene.xml'
    xml = ET.fromstring(xmlpath.read_text())
    assert hashlib.sha256(xmlpath.read_bytes()).hexdigest() == declaration['parent_model']['model_xml_sha256']
    params = {e.get('name'): {c.get('key'): float(c.get('value')) for c in e}
        for e in xml.findall('.//extension/plugin/instance')}
    male, female = params['bolt_shape'], params['nut_shape']
    pitch = male['pitch']
    assert female['pitch'] == pitch
    cold_chain = None
    if state_parent is not None:
        from independent_cold_chain_audit import verify_cold_chain
        cold_chain = verify_cold_chain(path, parent, state_parent, yaw_note=yaw_note)
        from independent_yaw_counter_origin import verify_recorded_counter_origin
        cold_chain = verify_recorded_counter_origin(path, cold_chain)
    pz = raw['base_z_m']
    yaw = raw['yaw_unwrapped_rad']+(cold_chain['actual_relative_yaw_equals_raw_plus_constant_rad'] if cold_chain else 0.)
    phase = wrap_phase(-pz+pitch*yaw/(2*np.pi)+male.get('phase', 0.)-female.get('phase', 0.), pitch)
    nominal = pz+male['length']+female['length']/2
    clearance = (female['pitch_diameter']-male['pitch_diameter'])/2
    span_threshold = male['chamfer']+female['chamfer']+5*np.sqrt(3.)*pitch/16
    initial = 0
    candidates = dict(initial=initial, final=len(pz)-1,
        closest_phase0=int(np.argmin(abs(phase))), deepest=int(np.argmax(nominal)),
        most_withdrawn=int(np.argmin(nominal)))
    selected = {}
    for label, index in candidates.items():
        selected[label] = dict(index=index, time_s=float(raw['time_s'][index]),
            elapsed_s=float(raw['elapsed_s'][index]), base_z_m=float(pz[index]),
            yaw_unwrapped_rad=float(yaw[index]), phase_residual_m=float(phase[index]),
            nominal_overlap_m=float(nominal[index]),
            actual_formed_full_ring_overlap_m=float(raw['formed_flank_overlap_m'][index]),
            loaded_actual_interior_contact_count=int(raw['loaded_actual_interior_flank_contact_count'][index]),
            profile_maximum_radial_interference_lower_bound_m=profile_interference(
                phase[index], pitch, male['pitch_diameter']/2, female['pitch_diameter']/2))
    estimated_phase_fit = abs(phase) <= clearance/np.sqrt(3.)
    return dict(scope='Read-only original pre-integration measured phase/depth and sampled radial-profile geometry. No integration, forces, prescribed helix, or capture qualification. Actual interior contacts/full-ring geometry remain separate from coaxial bulk profile estimates.',
        source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        ledger_sha256=hashlib.sha256((path/'original_native_force_ledger.npz').read_bytes()).hexdigest(),
        archived_xml_sha256=hashlib.sha256(xmlpath.read_bytes()).hexdigest(),
        cold_chain=cold_chain,
        raw_counter_bytes_unchanged=True,
        phase_formula='wrap(-base_z+pitch*relative_yaw/(2*pi)+male_plugin_phase-female_plugin_phase)',
        zero_isosurface_sampling_error_bound_m=float(np.sqrt(3.)*pitch/16384),
        nominal_cone_coincidence_overlap_m=clearance+male['chamfer']+female['chamfer'],
        common_unchamfered_axial_span_overlap_threshold_m=span_threshold,
        actual_base_z_range_m=[float(pz.min()), float(pz.max())],
        nominal_overlap_range_m=[float(nominal.min()), float(nominal.max())],
        actual_yaw_range_rad=[float(yaw.min()), float(yaw.max())],
        maximum_withdrawal_from_initial_m=float(pz[0]-pz.min()),
        actual_loaded_interior_contact_substeps=int(np.sum(raw['loaded_actual_interior_flank_contact_count'] > 0)),
        actual_positive_full_ring_overlap_substeps=int(np.sum(raw['formed_flank_overlap_m'] > 0)),
        estimated_coaxial_phase_fit_substeps=int(estimated_phase_fit.sum()),
        estimated_phase_fit_and_sufficient_nominal_depth_substeps=int(np.sum(
            estimated_phase_fit & (nominal > span_threshold))),
        selected_original_native_samples=selected, capture_qualified=False)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('directory')
    parser.add_argument('--output', required=True)
    parser.add_argument('--parent')
    parser.add_argument('--state-parent')
    parser.add_argument('--yaw-note')
    args = parser.parse_args()
    result = audit(args.directory, args.parent, args.state_parent, args.yaw_note)
    Path(args.output).write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print(json.dumps({key: result[key] for key in ['nominal_overlap_range_m',
        'maximum_withdrawal_from_initial_m', 'actual_loaded_interior_contact_substeps',
        'estimated_phase_fit_and_sufficient_nominal_depth_substeps',
        'selected_original_native_samples']}, indent=2))
