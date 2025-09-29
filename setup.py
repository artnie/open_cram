from setuptools import setup

package_name = 'open_cram'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/open_cram_launch.py']),
    ],
    install_requires=[
        'setuptools',
        # HTTP client for REST to Open-WebUI
        'httpx>=0.27',
        # OpenAI client (adjust if you use another library)
        'openai>=1.40',
    ],
    zip_safe=True,
    maintainer='Arthur Niedzwiecki',
    maintainer_email='arthur.niedzwiecki+github@gmail.com',
    description='ROS2 bridge between openAI, open-WebUI and CRAM applications.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'open_cram_node = open_cram.node:main',
        ],
    },
)