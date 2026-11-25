"""Robot description helpers shared by the launch files: namespaced xacro and the SDF to spawn."""
import os
import re
import subprocess
import tempfile

import xacro


def robot_description(xacro_file, gz_ns, profile='full'):
    """Return the URDF text: Gazebo topics prefixed by `gz_ns` ('' = none), sensor profile."""
    return xacro.process_file(xacro_file, mappings={'gz_ns': gz_ns,
                                                    'sensor_profile': profile}).toxml()


def build_sdf(xacro_file, gz_ns, profile='full', prefix='mecanum_forklift_'):
    """
    Convert the namespaced URDF to an SDF file and return its path.

    Same conversion as mecanum_forklift_gazebo.launch.py: the URDF->SDF converter drops the
    `ignition:expressed_in` attribute of <fdir1>, without which the wheels' anisotropic friction
    direction is fixed to the spinning wheel frame and the robot cannot strafe, so it is added back
    (frame = the chassis) before spawning.
    """
    tmp = tempfile.mkdtemp(prefix=prefix)
    urdf_path = os.path.join(tmp, 'robot.urdf')
    sdf_path = os.path.join(tmp, 'robot.sdf')
    with open(urdf_path, 'w', encoding='utf-8') as f:
        f.write(robot_description(xacro_file, gz_ns, profile))
    sdf = subprocess.run(['ign', 'sdf', '-p', urdf_path], check=True, capture_output=True,
                         text=True).stdout
    chassis = re.search(r"<link name='([^']+)'", sdf).group(1)
    sdf = re.sub(r"<model name='([^']+)'>",
                 r"<model name='\1' xmlns:ignition='http://ignitionrobotics.org/schema'>", sdf,
                 count=1)
    sdf = sdf.replace('<fdir1>', f"<fdir1 ignition:expressed_in='{chassis}'>")
    with open(sdf_path, 'w', encoding='utf-8') as f:
        f.write(sdf)
    return sdf_path
