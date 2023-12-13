import os

from glob import glob
from setuptools import find_packages, setup

package_name = "jinbot_core"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name), glob("launch/*launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="entity014",
    maintainer_email="unkn9wz@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "drive_node = jinbot_core.bot_drive:main",
            "flag_node = jinbot_core.bot_flaggripper:main",
            "joy_node = jinbot_core.joy_drive:main",
        ],
    },
)
