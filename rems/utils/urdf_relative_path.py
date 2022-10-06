import xml.etree.ElementTree as ET
from pathlib import Path

MESH = 'mesh'
FILENAME = 'filename'
TEMP = 'temp_'

def urdf_filepath_resolver(urdf_path, mesh_path):
    tree = ET.parse(urdf_path)
    mesh_path = Path(mesh_path)
    root = tree.getroot()
    for child in root.iter(MESH):
        filepath = Path(child.get(FILENAME))
        mesh_file_path = mesh_path.joinpath(filepath)
        child.set(FILENAME, str(mesh_file_path.absolute()))
    urdf = Path(urdf_path)
    temp_path = urdf.parent.joinpath(TEMP+urdf.name)
    tree.write(temp_path)
    return str(temp_path.absolute())