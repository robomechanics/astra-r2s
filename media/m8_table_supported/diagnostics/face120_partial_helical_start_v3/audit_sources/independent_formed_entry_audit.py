"""Separate formed geometry, partial native flank normals and stopped loading."""
from pathlib import Path
import argparse
import hashlib
import json
import xml.etree.ElementTree as ET

import numpy as np


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def normal_pitch_world_z(contact):
    """Infer helical slope from an ORIGINAL normal; axis/origin approximation explicit."""
    point = np.asarray(contact['contact_position_world_m'])-contact['bolt_origin_world_m']
    radius = float(np.linalg.norm(point[:2]))
    if radius <= 0:
        raise ValueError('Cylindrical normal requires nonzero measured radius')
    tangent = np.array([-point[1], point[0], 0.])/radius
    normal = np.asarray(contact['frame'])[0]
    azimuth, axial = float(normal@tangent), float(normal[2])
    return dict(radius_m=radius, normal_azimuth_component_world_Z=azimuth,
        normal_axial_component_world_Z=axial,
        estimated_pitch_from_normal_m=-2*np.pi*radius*azimuth/axial if abs(axial)>1e-12 else None,
        approximate_contact_male_z_m=float(-point[2]))


def audit(path):
    path = Path(path)
    raw = np.load(path/'original_native_force_ledger.npz', allow_pickle=False)
    saved = np.load(path/'checkpoint_trace.npz', allow_pickle=False)
    rows = json.loads(str(saved['info_json'].item()))
    declaration = json.loads((path/'declaration.json').read_text())
    xml = ET.fromstring((path/'scene.xml').read_text())
    plugins = {e.get('name'): {c.get('key'): float(c.get('value')) for c in e}
        for e in xml.findall('.//extension/plugin/instance')}
    pitch = plugins['bolt_shape']['pitch']
    dt = float(xml.find('option').get('timestep'))
    count = round(.1/dt)
    assert len(raw['time_s']) >= count
    window = slice(len(raw['time_s'])-count, len(raw['time_s']))
    geometry = raw['formed_flank_overlap_m'] > 0
    interior = raw['loaded_actual_interior_flank_contact_count'] > 0
    names = declaration['phase_names']
    turn = raw['phase_index'] == names.index('start_thread_1')
    formed_turn = np.flatnonzero(geometry & turn)
    lead = dict(observed=len(formed_turn)>1, actual_loaded_interior_substeps=int(np.sum(interior & turn)),
        scope='Geometry-positive native entry interval only. Without original loaded full-flank contacts and a qualified turn span, pitch consistency alone is not capture or qualified lead.')
    if len(formed_turn)>1:
        first, last = formed_turn[0], formed_turn[-1]
        theta = raw['yaw_unwrapped_rad'][last]-raw['yaw_unwrapped_rad'][first]
        advance = raw['base_z_m'][last]-raw['base_z_m'][first]
        expected = pitch*theta/(2*np.pi)
        lead.update(first_original_native_index=int(first), last_original_native_index=int(last),
            first_solved_state_time_s=float(raw['time_s'][first]-dt), last_solved_state_time_s=float(raw['time_s'][last]-dt),
            actual_yaw_span_rad=float(theta), actual_axial_advance_m=float(advance),
            ideal_pitch_comparison_only_m=float(expected), endpoint_error_from_pitch_m=float(advance-expected),
            inferred_pitch_over_this_short_span_m=float(2*np.pi*advance/theta) if theta else None,
            spans_full_pitch_rotation=bool(theta>=2*np.pi), qualified_lead=False)
    stopped = ((abs(raw['relative_bolt_axial_velocity_m_per_s'][window]) <= .0002)
        & (raw['relative_bolt_angular_speed_rad_per_s'][window] <= .01)
        & (raw['relative_hand_angular_speed_rad_per_s'][window] <= .01)
        & (raw['all_checks_held'][window] == 1))
    force, positive_hand = raw['thread_gravity_opposing_force_N'][window], np.maximum(raw['hand_gravity_opposing_force_N'][window], 0.)
    weight = declaration['bolt_weight_N']
    loaded_normals = []
    for row in rows:
        if row['time_s'] < raw['time_s'][-count]-1e-9:
            continue
        for c in row['native_contact_records']:
            if c['local_force_N_Nm'][0] <= 1e-5:
                continue
            normal = normal_pitch_world_z(c)
            normal['original_normal_force_N'] = c['local_force_N_Nm'][0]
            normal['producer_conservative_full_interior_tag'] = c['is_actual_interior_flank_contact']
            normal['loose_axis_centroid_uncertainty_context_rad'] = float(row['bolt_tilt_rad']+row['block_rotation_rad']+row['radial_offset_m']/normal['radius_m'])
            loaded_normals.append(normal)
    normal_ranges = {key: [min(row[key] for row in loaded_normals), max(row[key] for row in loaded_normals)]
        for key in ['normal_azimuth_component_world_Z', 'normal_axial_component_world_Z',
            'estimated_pitch_from_normal_m', 'loose_axis_centroid_uncertainty_context_rad']}
    assert sha(path/'scene.xml') == declaration['parent_model']['model_xml_sha256']
    return dict(scope='Original matched pre-integration body measurements/contact normals, with no native force reconstruction or integration. Body-state timestamp=logged_time−dt; saved qpos is post-integration. Native partial helical contacts remain distinct from conservative full-interior tags and full-thread qualification.',
        auditor_source_sha256=sha(__file__), input_sha256={name:sha(path/name) for name in
            ['original_native_force_ledger.npz','checkpoint_trace.npz','scene.xml','declaration.json']},
        original_native_steps=len(raw['time_s']), formed_geometry_substeps=int(np.sum(geometry)),
        actual_loaded_full_interior_substeps=int(np.sum(interior)),
        final_exact100ms=dict(native_samples=count, observed_duration_s=count*dt,
            every_native_sample_actually_stopped_and_all_original_guards=bool(stopped.all()),
            maximum_actual_bolt_angular_speed_rad_per_s=float(raw['relative_bolt_angular_speed_rad_per_s'][window].max()),
            maximum_actual_hand_angular_speed_rad_per_s=float(raw['relative_hand_angular_speed_rad_per_s'][window].max()),
            maximum_absolute_actual_bolt_axial_speed_m_per_s=float(abs(raw['relative_bolt_axial_velocity_m_per_s'][window]).max()),
            mean_signed_thread_weight_fraction=float(force.mean()/weight),
            mean_positive_hand_weight_fraction=float(positive_hand.mean()/weight),
            original_upward_thread_loaded_duty=float(np.mean(force>.1*weight)),
            formed_geometry_range_m=[float(raw['formed_flank_overlap_m'][window].min()),float(raw['formed_flank_overlap_m'][window].max())],
            actual_loaded_full_interior_substeps=int(np.sum(interior[window])),
            measured_stopped_weight_transfer_ready=bool(stopped.all() and force.mean()>=.9*weight
                and positive_hand.mean()<=.1*weight and force[-1]>.1*weight)),
        original_loaded_entry_normals=dict(original_sampled_positive_normal_contacts=len(loaded_normals),
            ranges=normal_ranges, all_conservative_full_interior_tags_false=all(not r['producer_conservative_full_interior_tag'] for r in loaded_normals),
            scope='Nonzero native normal azimuthal slope and ~.865 axial component provide helical-flank evidence. WorldZ cylinder uses nearly aligned native pose; inferred pitch and uncertainty are geometric approximations, not an independent replacement of exact contact-point body transforms. Entry tag means outside conservative full-flank spans, not necessarily a pure axisymmetric cone.'),
        geometric_entry_turn=lead,
        cumulative_original_grip_minimum_load_N={side:[float(raw[f'{side}_pad_{i}_N'].min()) for i in (0,1)] for side in ('left','right')},
        cumulative_original_grip_slip_maximum_m={side:float(raw[f'{side}_grip_slip_m'].max()) for side in ('left','right')},
        cumulative_original_grip_rotation_slip_maximum_rad={side:float(raw[f'{side}_grip_rotation_slip_rad'].max()) for side in ('left','right')},
        capture_qualified=False, unsupported_reset_qualified=False, full_trajectory_qualified=False)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('directory')
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    result = audit(args.directory)
    Path(args.output).write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps(result,indent=2))
