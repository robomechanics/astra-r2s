"""Recordings must survive relocation while rejecting changed scene inputs."""
from dataclasses import replace
import hashlib
from pathlib import Path
import shutil

from yam_twin import m8_scene


def test_scene_identity_survives_checkout_relocation_but_detects_mesh_changes(tmp_path, monkeypatch):
    original = m8_scene.YAM_XML
    config = m8_scene.YamM8Config()
    identity = m8_scene.scene_fingerprint(config)
    literal = hashlib.sha256(m8_scene.scene_xml(config).encode()).hexdigest()
    relocated = tmp_path / "relocated" / "xmls"
    relocated.mkdir(parents=True)
    shutil.copy2(original, relocated / original.name)
    shutil.copytree(original.parent / "assets", relocated / "assets")
    monkeypatch.setattr(m8_scene, "YAM_XML", relocated / original.name)
    assert hashlib.sha256(m8_scene.scene_xml(config).encode()).hexdigest() != literal
    assert m8_scene.scene_fingerprint(config) == identity
    mesh = relocated / "assets" / "model2.stl"
    mesh.write_bytes(mesh.read_bytes() + b"modified asset")
    assert m8_scene.scene_fingerprint(config) != identity


def test_scene_identity_detects_fixture_and_contact_parameter_changes():
    config = m8_scene.YamM8Config()
    identity = m8_scene.scene_fingerprint(config)
    assert m8_scene.scene_fingerprint(replace(config, bolt_position=(.301, -.06, .1025))) != identity
    assert m8_scene.scene_fingerprint(replace(config, thread=replace(config.thread, friction=.2))) != identity
