"""Pruebas de control_bebop.paths con un workspace falso (sin ROS)."""
import os
import sys

import pytest

from control_bebop import paths

PKG_XML = '<package format="3"><name>control_bebop</name></package>'


def _fake_ws(tmp_path, layout='src/Vuelo_bebop_v1/control_bebop'):
    ws = tmp_path / 'ros2_ws'
    pkg = ws / layout
    pkg.mkdir(parents=True)
    (pkg / 'package.xml').write_text(PKG_XML)
    (pkg / 'setup.py').write_text('')
    (pkg / 'config').mkdir()
    prefix = ws / 'install' / 'control_bebop'
    (prefix / 'share' / 'control_bebop' / 'config').mkdir(parents=True)
    # Un paquete con otro nombre que no debe confundirse
    other = ws / 'src' / 'otro_pkg'
    other.mkdir(parents=True)
    (other / 'package.xml').write_text('<package><name>otro_pkg</name></package>')
    (other / 'setup.py').write_text('')
    return ws, pkg, prefix


@pytest.fixture
def fake(tmp_path, monkeypatch):
    def make(layout='src/Vuelo_bebop_v1/control_bebop'):
        ws, pkg, prefix = _fake_ws(tmp_path, layout)
        monkeypatch.delenv('CONTROL_BEBOP_DIR', raising=False)
        monkeypatch.delenv('BEBOP_DATA_DIR', raising=False)
        monkeypatch.setattr(paths, '_install_prefix', lambda: str(prefix))
        monkeypatch.setattr(paths, 'share_dir',
                            lambda: str(prefix / 'share' / 'control_bebop'))
        # Simular instalación normal (no symlink): __file__ fuera de src
        monkeypatch.setattr(paths, '__file__', str(prefix / 'lib' / 'x' / 'paths.py'))
        paths.package_source_dir.cache_clear()
        return ws, pkg, prefix
    yield make
    paths.package_source_dir.cache_clear()


@pytest.mark.parametrize('layout', [
    'src/Vuelo_bebop_v1/control_bebop',   # repo clonado dentro de src
    'src/control_bebop',                  # paquete directo en src
])
def test_finds_source_in_any_layout(fake, layout):
    ws, pkg, _ = fake(layout)
    assert paths.package_source_dir() == str(pkg)
    assert paths.workspace_dir() == str(ws)


def test_data_dirs_are_inside_package_and_created(fake):
    _, pkg, _ = fake()
    for sub in ('bags', 'videos', 'trayectorias'):
        d = paths.data_dir(sub)
        assert d == os.path.join(str(pkg), sub)
        assert os.path.isdir(d)


def test_env_overrides(fake, tmp_path, monkeypatch):
    _, pkg, _ = fake()
    monkeypatch.setenv('BEBOP_DATA_DIR', str(tmp_path / 'datos'))
    assert paths.data_dir('bags') == str(tmp_path / 'datos' / 'bags')
    monkeypatch.setenv('CONTROL_BEBOP_DIR', str(pkg))
    paths.package_source_dir.cache_clear()
    assert paths.package_source_dir() == str(pkg)


def test_config_file_prefers_source(fake):
    _, pkg, prefix = fake()
    name = 'real_drone_calibrated.yaml'
    assert paths.config_file(name) is None
    share_file = prefix / 'share' / 'control_bebop' / 'config' / name
    share_file.write_text('a: 1')
    assert paths.config_file(name) == str(share_file)
    src_file = pkg / 'config' / name
    src_file.write_text('a: 2')
    assert paths.config_file(name) == str(src_file)
    assert paths.source_config_path(name) == str(src_file)


def test_fallback_when_source_missing(tmp_path, monkeypatch):
    monkeypatch.delenv('CONTROL_BEBOP_DIR', raising=False)
    monkeypatch.delenv('BEBOP_DATA_DIR', raising=False)
    monkeypatch.setenv('HOME', str(tmp_path))
    monkeypatch.setattr(paths, '_install_prefix', lambda: None)
    monkeypatch.setattr(paths, '__file__', str(tmp_path / 'x' / 'y' / 'paths.py'))
    paths.package_source_dir.cache_clear()
    with pytest.warns(UserWarning):
        d = paths.data_dir('videos')
    assert d == str(tmp_path / '.ros' / 'control_bebop' / 'videos')
    paths.package_source_dir.cache_clear()


def test_venv_only_matching_python(tmp_path, monkeypatch):
    monkeypatch.setenv('HOME', str(tmp_path))
    ver = f'python{sys.version_info.major}.{sys.version_info.minor}'
    good = tmp_path / 'rob_env' / 'lib' / ver / 'site-packages'
    wrong = tmp_path / 'rob_env' / 'lib' / 'python2.7' / 'site-packages'
    good.mkdir(parents=True)
    wrong.mkdir(parents=True)
    monkeypatch.setattr(sys, 'path', list(sys.path))
    added = paths.add_user_venvs(('rob_env',))
    assert added == [str(good)]
    assert str(wrong) not in sys.path
