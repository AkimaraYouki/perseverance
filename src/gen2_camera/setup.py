from setuptools import setup
setup(
    name="gen2_camera", version="0.1.0", packages=["gen2_camera"],
    data_files=[("share/ament_index/resource_index/packages", ["resource/gen2_camera"]),
                ("share/gen2_camera", ["package.xml"]),
                ("share/gen2_camera/launch", ["launch/camera.launch.py"]),
                ("share/gen2_camera/config", ["config/camera.yaml"])],
    install_requires=["setuptools"], zip_safe=True,
    entry_points={"console_scripts": ["camera_node = gen2_camera.camera_node:main",
                                    "camera_latency = gen2_camera.latency_probe:main"]},
)
