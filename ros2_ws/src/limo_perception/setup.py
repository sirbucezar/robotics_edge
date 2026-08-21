from setuptools import setup

package_name = "limo_perception"

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
    description="Person detection nodes for the LIMO Pro perception-driven navigation project.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mock_detector_node = limo_perception.mock_detector_node:main",
            "yolo_detector_node = limo_perception.yolo_detector_node:main",
            "color_camera_node = limo_perception.color_camera_node:main",
        ],
    },
)
