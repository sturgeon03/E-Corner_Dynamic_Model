from pathlib import Path

from setuptools import find_packages, setup


PACKAGE_NAME = "vehicle_sim"
PACKAGE_ROOT = Path(__file__).parent / PACKAGE_NAME


def _collect_package_data() -> list[str]:
    patterns = ("*.yaml", "*.yml", "*.csv")
    data_files: list[str] = []
    for pattern in patterns:
        for path in PACKAGE_ROOT.rglob(pattern):
            data_files.append(str(path.relative_to(PACKAGE_ROOT)).replace("\\", "/"))
    return sorted(set(data_files))

setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=find_packages(),
    package_data={PACKAGE_NAME: _collect_package_data()},
    include_package_data=True,
    python_requires=">=3.10",
    install_requires=[
        'numpy',
        'pyyaml',
        'matplotlib',
    ],
)
