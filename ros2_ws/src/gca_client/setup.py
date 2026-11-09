from setuptools import find_packages, setup

setup(
    name="gca_client",
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/gca_client"]),
        ("share/gca_client", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="GCA maintainers",
    maintainer_email="maintainers@example.invalid",
    description="Upper-host motion client without device-side dependencies",
    license="LicenseRef-Project-Unspecified",
    entry_points={"console_scripts": ["gca_client = gca_client.cli:main"]},
)
