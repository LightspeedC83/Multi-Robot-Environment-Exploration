from glob import glob
from setuptools import find_packages, setup

package_name = "final_project_cv"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}", ["requirements_ros.txt"]),
        (f"share/{package_name}/models", glob("*.pt")),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/rviz", glob("rviz/*.rviz")),
        (f"share/{package_name}/worlds", glob("worlds/*.world")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Emilio Daza",
    maintainer_email="emilio.daza@dartmouth.edu",
    description="Computer vision target localization utilities for the final project.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "centroid_test_source = final_project_cv.centroid_test_source_node:main",
            "target_localizer = final_project_cv.target_localizer_node:main",
            "target_trace_recorder = final_project_cv.target_trace_recorder_node:main",
            "video_source = final_project_cv.video_source_node:main",
            "vision_target_detector = final_project_cv.vision_target_detector_node:main",
        ],
    },
)
