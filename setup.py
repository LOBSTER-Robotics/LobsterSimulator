import setuptools

setuptools.setup(
    name='lobster_simulator',
    version='0.0.2',
    description='Simulator for the Lobster uuv',
    url='https://github.com/LOBSTER-Robotics/LobsterSimulator',
    author='Joris Quist',
    author_email='Jorisquist@gmail.com',
    license='',
    packages= setuptools.find_packages(),
    include_package_data=True,
    install_requires=['pybullet',
                      'numpy'
                      'inputs'
                      ],

    classifiers=[],
)

