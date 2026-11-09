from setuptools import find_packages, setup

setup(
    name="gca_position",
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/gca_position"]),
        ("share/gca_position", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="GCA project contributors",
    maintainer_email="maintainer@example.invalid",
    description="External encoder position feedback and PID strategy without ROS imports.",
    license="LicenseRef-Project-Unspecified",
)
