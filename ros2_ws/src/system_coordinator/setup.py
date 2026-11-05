from setuptools import find_packages, setup

setup(
    name="system_coordinator",
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/system_coordinator"]),
        ("share/system_coordinator", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="GCA maintainers",
    maintainer_email="maintainers@example.invalid",
    description="Unique source control authority and GCA-only task coordinator",
    license="LicenseRef-Project-Unspecified",
    entry_points={"console_scripts": ["system_coordinator = system_coordinator.node:main"]},
)
