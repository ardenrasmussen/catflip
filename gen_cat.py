import numpy as np

def generate_urdf(args):
    file = open("robocat.urdf", 'w')
    ixx = 1/12*args.mass*(args.radius**2+args.length**2)
    iyy = 1/12*args.mass*(args.radius**2+args.length**2)
    izz = 1/2*args.mass*args.radius**2

    frx, fry, frz = np.pi / 2.0, 0.0, 0.0
    ftx, fty, ftz = 0.0, args.length/2.0 + 0, 0.0
    brx, bry, brz = np.pi / 2.0, 0.0, 0.0
    btx, bty, btz = 0.0, -args.length/2.0 - 0, 0.0
    file.write("""
    <?xml version="1.0"?>
    <robot name="robocat">
        <link name="frontCylinder">
        <visual>
            <geometry>
                <cylinder length="{length}" radius="{radius}" />
            </geometry>
            <origin rpy="{frx} {fry} {frz}" xyz="{ftx} {fty} {ftz}" />
            <material name="gen-mat">
                <color rgba="1.0 1.0 1.0 1.0" />
            </material>
        </visual>
        <collision>
            <geometry>
                <cylinder length="{length}" radius="{radius}" />
            </geometry>
            <origin rpy="{frx} {fry} {frz}" xyz="{ftx} {fty} {ftz}" />
        </collision>
        <inertial>
            <mass value="{mass}" />
            <inertia ixx="{ixx}" ixy="0.0" ixz="0.0" iyy="{iyy}" iyz="0.0" izz="{izz}" />
            <origin rpy="{frx} {fry} {frz}" xyz="{ftx} {fty} {ftz}" />
        </inertial>
        </link>
        <link name="frontCylinder-leg">
        <visual>
            <geometry>
                <cylinder length="0.5" radius="0.1" />
            </geometry>
            <material name="gen-mat">
                <color rgba="1.0 1.0 1.0 1.0" />
            </material>
            <origin rpy="0.0 0.0 0.0" xyz="0.0 {fty} {radius}" />
        </visual>
        </link>
        <joint name="frontCylinder-leg-to-frontCylinder" type="fixed">
            <parent link="frontCylinder" />
            <child link="frontCylinder-leg" />
            <origin xyz="0.0 {fty} {radius}" />
        </joint>
        <link name="backCylinder">
        <visual>
            <geometry>
                <cylinder length="{length}" radius="{radius}" />
            </geometry>
            <origin rpy="{brx} {bry} {brz}" xyz="{btx} {bty} {btz}" />
            <material name="gen-mat">
                <color rgba="1.0 1.0 1.0 1.0" />
            </material>
        </visual>
        <collision>
            <geometry>
                <cylinder length="{length}" radius="{radius}" />
            </geometry>
            <origin rpy="{brx} {bry} {brz}" xyz="{btx} {bty} {btz}" />
        </collision>
        <inertial>
            <mass value="{mass}" />
            <inertia ixx="{ixx}" ixy="0.0" ixz="0.0" iyy="{iyy}" iyz="0.0" izz="{izz}" />
            <origin rpy="{brx} {bry} {brz}" xyz="{btx} {bty} {btz}" />
        </inertial>
        </link>
        <link name="backCylinder-leg">
        <visual>
            <geometry>
                <cylinder length="0.5" radius="0.1" />
            </geometry>
            <material name="gen-mat">
                <color rgba="1.0 1.0 1.0 1.0" />
            </material>
            <origin rpy="0.0 0.0 0.0" xyz="0.0 {bty} {radius}" />
        </visual>
        </link>
        <joint name="backCylinder-leg-to-backCylinder" type="fixed">
            <parent link="backCylinder" />
            <child link="backCylinder-leg" />
            <origin xyz="0.0 {bty} {radius}" />
        </joint>
        <link name="centerSphere">
        <visual>
            <geometry>
                <sphere radius="{radius}" />
            </geometry>
            <material name="gen-mat">
                <color rgba="1.0 1.0 1.0 1.0" />
            </material>
            <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0" />
        </visual>
        </link>
        <joint name="front_to_center" type="revolute">
            <axis xyz="1 0 0" />
            <limit effort="0.0" lower="-3.1415" upper="3.1415" velocity="1000.0" />
            <parent link="centerSphere" />
            <child link="frontCylinder" />
            <origin xyz="0 0 0" />
        </joint>
        <joint name="back_to_center" type="revolute">
            <axis xyz="0 0 1" />
            <limit effort="0.0" lower="-3.1415" upper="3.1415" velocity="1000.0" />
            <parent link="centerSphere" />
            <child link="backCylinder" />
            <origin xyz="0 0 0" />
        </joint>
    </robot>
    """.format(mass=args.mass,length=args.length,radius=args.radius,ixx=ixx,iyy=iyy,izz=izz,frx=frx,fry=fry,frz=frz,ftx=ftx,fty=fty,ftz=ftz,brx=brx,bry=bry,brz=brz,btx=btx,bty=bty,btz=btz))
    file.close()
