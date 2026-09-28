from glob import glob
import os

from setuptools import setup


package_name = 'r1_neupan_integration'


def package_files(directory):
    data_files = []
    for root, _, files in os.walk(directory):
        if files:
            relative = os.path.relpath(root, '.')
            data_files.append((
                os.path.join('share', package_name, relative),
                [os.path.join(root, filename) for filename in files],
            ))
    return data_files


setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ] + package_files('config') + package_files('worlds'),
    scripts=['scripts/n1_6_benchmark.py', 'scripts/n2_pre_safety_unit.py',
             'scripts/n2_pre_semantics_unit.py', 'scripts/n2_scan_self_filter_unit.py',
             'scripts/n2_forward_only_report.py',
             'scripts/n2_1_geometry_audit.py',
             'scripts/n2_1_failure_attribution_report.py',
             'scripts/n2_2_failure_attribution.py',
             'scripts/n2_3_contract_unit.py', 'scripts/n2_3_report.py',
             'scripts/n2_3r1_live_fault.py', 'scripts/n2_3r1_report.py',
             'scripts/n2_4_recovery_unit.py', 'scripts/n2_4_shadow.py',
             'scripts/n2_4_candidate_replay.py', 'scripts/n2_4_exact_replay.py',
             'scripts/n2_4c_reproducibility_report.py',
             'scripts/n2_4c2_matrix_report.py',
             'scripts/n2_4c_runtime_report.py'],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'fixed_path_publisher = r1_neupan_integration.fixed_path_publisher:main',
            'n1_metrics = r1_neupan_integration.n1_metrics:main',
            'n1_5_core_ab = r1_neupan_integration.n1_5_core_ab:main',
            'n1_5_config_diff = r1_neupan_integration.n1_5_config_diff:main',
            'n1_15_node = r1_neupan_integration.n1_15_node:main',
            'n1_16_node = r1_neupan_integration.n1_16_node:main',
            'n2_3_command_relay = r1_neupan_integration.n2_3_command_relay:main',
            'r1_safety_gate = r1_neupan_integration.n2_pre_safety_gate:main',
            'n2_pre_scenarios = r1_neupan_integration.n2_pre_scenarios:main',
            'n2_pre_audit = r1_neupan_integration.n2_pre_audit:main',
            'n2_pre3_audit = r1_neupan_integration.n2_pre3_audit:main',
            'n2_pre3_sensor_relay = r1_neupan_integration.n2_pre3_sensor_relay:main',
            'n2_pre_no_reverse_probe = r1_neupan_integration.n2_pre_no_reverse_probe:main',
            'n2_4c_runtime_attribution = r1_neupan_integration.n2_4c_runtime_attribution:main',
            'cleanup_n1_world = r1_neupan_integration.cleanup_n1_world:main',
        ],
    },
)
