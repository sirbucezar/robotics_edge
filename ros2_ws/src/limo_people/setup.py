from setuptools import setup

package_name = "limo_people"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Cezar",
    maintainer_email="sirbum605@gmail.com",
    description="Map-frame people tracking and counting for the LIMO Pro project.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "people_tracker_node = limo_people.people_tracker_node:main",
        ],
    },
)
