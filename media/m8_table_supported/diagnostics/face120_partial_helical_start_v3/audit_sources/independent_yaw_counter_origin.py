"""Recognize archived counter initialization and verify its cold-pose origin."""
import ast
from pathlib import Path

import numpy as np


def initial_assignment(source, name):
    rows = [node for node in ast.walk(ast.parse(source)) if isinstance(node, ast.Assign)
        and any(isinstance(target, ast.Name) and target.id == name for target in node.targets)]
    if not rows:
        raise ValueError(f'No archived initialization for {name}')
    return min(rows, key=lambda row: row.lineno).value


def classify_counter_origin(source):
    value = initial_assignment(source, 'last_yaw')
    if (isinstance(value, ast.Subscript) and isinstance(value.value, ast.Name)
            and value.value.id == 'old' and isinstance(value.slice, ast.Constant)
            and value.slice.value == 'bolt_yaw_unwrapped_rad'):
        return 'canonical_parent_counter'
    if isinstance(value, ast.Name) and value.id == 'wrapped_last':
        value = initial_assignment(source, 'wrapped_last')
    names = {node.id for node in ast.walk(value) if isinstance(node, ast.Name)}
    attrs = {node.attr for node in ast.walk(value) if isinstance(node, ast.Attribute)}
    if {'data', 'hr'} <= names and {'arctan2', 'xmat'} <= attrs:
        return 'actual_cold_state_wrapped_yaw'
    raise ValueError('Unknown archived absolute-yaw initialization; do not infer a correction')


def verify_recorded_counter_origin(path, chain):
    path = Path(path)
    origin = classify_counter_origin((path/'diagnostic_source.py').read_text())
    raw = np.load(path/'original_native_force_ledger.npz', allow_pickle=False)
    first = float(raw['yaw_unwrapped_rad'][0])
    if origin == 'canonical_parent_counter':
        expected = chain['retained_canonical_counter_origin_rad']
        correction = chain['actual_relative_yaw_equals_raw_plus_constant_rad']
    else:
        expected = chain['actual_initial_relative_wrapped_yaw_rad']
        correction = 0.
    assert abs(first-expected) < 1e-4, 'Original first counter differs from independently derived declared initialization'
    result = dict(chain)
    result['potential_offset_if_canonical_origin_retained_rad'] = chain['actual_relative_yaw_equals_raw_plus_constant_rad']
    result['actual_relative_yaw_equals_raw_plus_constant_rad'] = correction
    result['archived_counter_origin'] = origin
    result['first_original_counter_vs_declared_origin_rad'] = first-expected
    result['counter_origin_auditor_source_sha256'] = chain_sha(__file__)
    return result


def chain_sha(path):
    import hashlib
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()
