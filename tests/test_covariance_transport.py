import pathlib
import subprocess
import tempfile
import unittest

class EigenTransport(unittest.TestCase):
    def test_installed_eigen_dense_nan_transport(self):
        source=pathlib.Path(__file__).with_name('mavros_covariance_eigen.cpp')
        with tempfile.TemporaryDirectory() as temp:
            binary=str(pathlib.Path(temp)/'covariance')
            subprocess.run(['c++','-O2','-I/usr/include/eigen3',str(source),'-o',binary],check=True,capture_output=True)
            subprocess.run([binary],check=True,capture_output=True)
