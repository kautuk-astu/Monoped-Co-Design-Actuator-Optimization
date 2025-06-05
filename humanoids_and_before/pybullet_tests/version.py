import pkg_resources

pybullet_version = pkg_resources.get_distribution("pybullet").version
print(pybullet_version)
