import ast
import os
import yaml
import pytest

CONFIG_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'config'))
LAUNCH_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'launch'))


@pytest.mark.parametrize('device,weighted,pairs', [
    ('g60', False, 30),
    ('g90', True, 200),
    ('d1m', True, 200),
])
def test_alignment_configs(device, weighted, pairs):
    path = os.path.join(CONFIG_DIR, f'{device}_alignment.yaml')
    assert os.path.exists(path), f"Missing config: {path}"
    with open(path, 'r') as f:
        data = yaml.safe_load(f)
    params = data['gnss_alignment']['ros__parameters']
    assert params['use_weighted_fit'] is weighted
    assert params['calibration_pairs'] == pairs
    assert params['transform_path'] == f'{device}_gps_odom_transform.txt'


@pytest.mark.parametrize('device,has_rtk', [
    ('g60', False),
    ('g90', True),
    ('d1m', True),
])
def test_transform_configs(device, has_rtk):
    path = os.path.join(CONFIG_DIR, f'{device}_transform.yaml')
    assert os.path.exists(path), f"Missing config: {path}"
    with open(path, 'r') as f:
        data = yaml.safe_load(f)
    params = data['gnss_transform']['ros__parameters']
    assert params['transform_path'] == f'{device}_gps_odom_transform.txt'
    if has_rtk:
        assert params['rtk_topic'] == '/rtk_pvh'
    else:
        assert params['rtk_topic'] == ''


@pytest.mark.parametrize('launch_file', [
    'alignment.launch.py',
    'transform.launch.py',
    'driver.launch.py',
])
def test_launch_files_syntax(launch_file):
    path = os.path.join(LAUNCH_DIR, launch_file)
    assert os.path.exists(path)
    with open(path, 'r') as f:
        tree = ast.parse(f.read(), filename=path)
    assert tree is not None
