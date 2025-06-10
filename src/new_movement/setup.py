from setuptools import find_packages, setup

package_name = "new_movement"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="fabio",
    maintainer_email="fabio_huang@ufms.br",
    description="This is the movement package for Ararabots SSL, "
    "it is responsible for calculating a sequence of points in space given a inital position and a final one.",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
