import numpy as np
import math
from omni.isaac.core_nodes import BaseResetNode
from omni.isaac.core.articulations import Articulation
from scipy.spatial.transform import Rotation as R
import torch
import torch.nn as nn
import io


def extract_yaw_from_quaternion(quat):
    w, x, y, z = quat
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return yaw

def rotate_vector(x, y, theta):
    x_new = x * math.cos(theta) - y * math.sin(theta)
    y_new = x * math.sin(theta) + y * math.cos(theta)
    return np.array([x_new, y_new])


# Please refer to the paper to understand the method implemented here at first on a high level.
class OmnidirectionalControllerInternalState(BaseResetNode):
    def __init__(self):
        self.robot_prim = None
        self.controller_handle = None
        super().__init__(initialize=False)

        self.last_linear_velocity_command = np.array([0.0, 0.0], dtype=np.double)
        self.last_angular_velocity_command = 0.
        self.time_since_last_command = 0.0

        self.prev_linear_velocity = np.array([0.0, 0.0], dtype=np.double)
        self.prev_angular_velocity = 0.

        self.curve_params = None
        self.curve_model = S_Curve_net()
        self.curve_model.load_state_dict(model_state_dict)
        self.curve_model.to("cpu")

        self.max_angular_speed = 9999.0
        self.max_linear_speed = 9999.0
        self.wheel_positions = np.array([0.244, 0.22317])
        self.wheel_radius = 0.0762

    def initialize_controller(self):
        self.controller_handle = Articulation(self.robot_prim)
        self.controller_handle.initialize()
        self.initialized = True

    def set_linear_velocity(self, linear_velocity_command, angular_velocity_command, dt, directlySetVelocity):
        global global_db
        # Set linear and angular velocity (X, Y, Z) on the robot in world frame
        if self.initialized:

            # Limit the angular velocity to the maximum allowed speed
            angular_velocity_command = np.clip(angular_velocity_command, -self.max_angular_speed, self.max_angular_speed)

            # Compute the total velocity along the circular trajectory
            # Total velocity of a wheel is sum of robot center velocity and the robots angular velocity times radius. This
            # can be shown to be:
            total_velocity = np.sqrt((abs(linear_velocity_command[0]) - abs(angular_velocity_command) * self.wheel_positions[1])**2 + (abs(linear_velocity_command[1]) + abs(angular_velocity_command) * self.wheel_positions[0])**2)

            # If total velocity exceeds max allowed speed, scale down both linear and angular velocity
            if total_velocity > self.max_linear_speed:
                scale = self.max_linear_speed / total_velocity
                linear_velocity_command[0] *= scale
                linear_velocity_command[1] *= scale
                angular_velocity_command *= scale

            # Check whether the command has changed significantly
            if not (np.linalg.norm(linear_velocity_command-self.last_linear_velocity_command)<1e-3 and np.abs(angular_velocity_command- self.last_angular_velocity_command) < 1e-3):
                #global_db.log_warn("New command: " + str(linear_velocity_command) + " " + str(angular_velocity_command))

                # Calculate how much percentage of the previous command has been done, and get the current velocity from that.
                if self.time_since_last_command > 0.0:
                    if not self.curve_params is None:
                        percentage = f(self.time_since_last_command, *self.curve_params) / self.curve_params[0]
                        #global_db.log_warn("percentage previous command: " + str(percentage))
                    else:
                        percentage = 1.0
                else:
                    percentage = 0.0
                cur_linear_velocity = self.prev_linear_velocity + percentage * (self.last_linear_velocity_command - self.prev_linear_velocity)
                cur_angular_velocity = self.prev_angular_velocity + percentage * (self.last_angular_velocity_command - self.prev_angular_velocity)

                # current_velocity = np.sqrt((abs(cur_linear_velocity[0]) - abs(cur_angular_velocity) * self.wheel_positions[1])**2 + (abs(cur_linear_velocity[1]) + abs(cur_angular_velocity) * self.wheel_positions[0])**2)
                # global_db.log_warn("current_velocity: " + str(current_velocity))


                # Use the total velocity change to get the S-curve parameters
                # This velocity change is the sum of the linear and angular velocity changes
                total_velocity_change_x = abs(linear_velocity_command[0]-cur_linear_velocity[0]) - (abs(angular_velocity_command-cur_angular_velocity)%(2*np.pi)) * self.wheel_positions[1]
                total_velocity_change_y = abs(linear_velocity_command[1]-cur_linear_velocity[1]) + (abs(angular_velocity_command-cur_angular_velocity)%(2*np.pi)) * self.wheel_positions[0]
                total_velocity_change = np.sqrt(total_velocity_change_x**2 + total_velocity_change_y**2) 
                total_velocity_change_wheel_reference = total_velocity_change / self.wheel_radius
                # global_db.log_warn("total_velocity_change: " + str(total_velocity_change))

                if total_velocity_change_wheel_reference < 1e-6:
                    self.curve_params = None
                else:
                    curve_params = self.curve_model.forward(L=total_velocity_change_wheel_reference) # Get fitted S-curve parameters
                    self.curve_params = total_velocity_change_wheel_reference, *curve_params
                    #global_db.log_warn("curve params:"+str(self.curve_params))
            
                self.time_since_last_command = 0.0                
                
                # Save the current commands
                self.last_linear_velocity_command = linear_velocity_command
                self.last_angular_velocity_command = angular_velocity_command

                # Save the current linear and angular velocity of the robot
                _, orientation_quat = self.controller_handle.get_world_pose()
                current_rotation = extract_yaw_from_quaternion(orientation_quat)
                cur_lin_vel = self.controller_handle.get_linear_velocity()[:2]
                cur_lin_vel = rotate_vector(cur_lin_vel[0], cur_lin_vel[1], -current_rotation)
                self.prev_linear_velocity = cur_lin_vel
                self.prev_angular_velocity = self.controller_handle.get_angular_velocity()[2]


            self.time_since_last_command += dt

            if not self.curve_params is None:
                # Linear interpolation between current velocities and commanded velocities, according to the rampup behavior of the original robot
                percentage = f(self.time_since_last_command, *self.curve_params) / self.curve_params[0]
                
            else:
                percentage = 1.0
            cur_linear_velocity = self.prev_linear_velocity + percentage * (self.last_linear_velocity_command - self.prev_linear_velocity)
            cur_angular_velocity = self.prev_angular_velocity + percentage * (self.last_angular_velocity_command - self.prev_angular_velocity)
            

            if directlySetVelocity: # If we don't use an articulation controller, but directly set the velocity
                # Set the linear and angular velocity on the robot

                # First rotate the linear velocity from robot frame to world frame
                _, orientation_quat = self.controller_handle.get_world_pose() # Get the robot's world pose (position and orientation as a quaternion)
                current_rotation = extract_yaw_from_quaternion(orientation_quat)
                linear_velocity_world = rotate_vector(cur_linear_velocity[0], cur_linear_velocity[1], current_rotation)

                # Set the linear and angular velocity on the robot
                self.controller_handle.set_angular_velocity(np.array([0.0, 0.0, cur_angular_velocity]) )
                self.controller_handle.set_linear_velocity(np.array([linear_velocity_world[0], linear_velocity_world[1], 0.0]))
            return cur_linear_velocity[0], -cur_linear_velocity[1], cur_angular_velocity

    def custom_reset(self):
        self.controller_handle = None

global global_db
class OgnOmnidirectionalVelocityController:
    """
    This class defines the behavior of the Omnidirectional Velocity Controller node. 
    It sets the robot's linear and angular velocities based on the inputs provided.
    """

    @staticmethod
    def internal_state():
        # This returns an instance of the internal state class
        return OmnidirectionalControllerInternalState()

    @staticmethod
    def compute(db) -> bool:
        global global_db
        """Compute the outputs from the current input"""

        state = db.internal_state

        try:
            # Initialization
            if not state.initialized:
                global_db = db
                # Check if we should use robot path or targetPrim
                if db.inputs.usePath:
                    state.robot_prim = db.inputs.robotPath
                else:
                    if len(db.inputs.targetPrim) == 0:
                        db.logError("Omnigraph Error: No robot prim found")
                        return False
                    else:
                        state.robot_prim = db.inputs.targetPrim[0].GetString()

                state.wheel_radius = db.inputs.wheelRadius
                state.wheel_positions = db.inputs.wheelPositions
                state.max_linear_speed = db.inputs.maxLinearSpeed
                state.max_angular_speed = db.inputs.maxAngularSpeed

                # Initialize the controller for the robot
                state.initialize_controller()

            # Gather input velocities
            linear_velocity_x = db.inputs.linearX
            linear_velocity_y = db.inputs.linearY
            angular_velocity_z = db.inputs.angularZ
            
            # Set linear and angular velocities using the controller
            x, y, z = state.set_linear_velocity(np.array([linear_velocity_x, linear_velocity_y], dtype=np.double), angular_velocity_z, db.inputs.deltaTime, db.inputs.directlySetVelocity)

            db.outputs.linearX = x
            db.outputs.linearY = y
            db.outputs.angularZ = z

        except Exception as error:
            db.log_warn(str(error))
            return False

        # Successful execution
        return True



########### Model of fitted S-curve velocity profile ###########


def softplus(x, sharpness):
    return nn.functional.softplus(x*sharpness) / sharpness

def inv_softplus(y, sharpness):
    input = y * sharpness
    is_tensor = True
    if not isinstance(input, torch.Tensor):
        is_tensor = False
        input = torch.tensor(input)
    out = input.expm1().clamp_min(1e-6).log()
    if not is_tensor and out.numel() == 1:
        return out.item()
    return out / sharpness

softplus_lin_point = 5
def softplus_startpart(x, a, target_slope, sharpness):
    return softplus((x*2*softplus_lin_point/a - softplus_lin_point), sharpness = sharpness) * target_slope / (2*softplus_lin_point) * a

def softplus_unshifted_endpart(x, b, target_slope, sharpness):
    return - softplus_startpart(-x, b, target_slope, sharpness)


def softplus_endpart(x, limit, a, b, target_slope, sharpness1, sharpness2):
    eps = 1e-5
    slope = (softplus_startpart(b, a, target_slope, sharpness1) - softplus_startpart(b-eps, a, target_slope, sharpness1))/eps
    slope = torch.max(slope, torch.tensor(0.1))
    x_lin = - softplus_startpart(b, a, target_slope, sharpness1) / slope + b
    x_end = - b/(2*softplus_lin_point) * (inv_softplus(limit / slope * (2*softplus_lin_point) / b, sharpness=sharpness2) + softplus_lin_point)
    shift = x_lin-x_end
    return limit + softplus_unshifted_endpart(x-shift, b, slope, sharpness2)

def f(x, limit, a, b, target_slope, sharpness1, sharpness2):
    limit = torch.tensor([limit]).float()
    a = torch.tensor([a]).float()
    b = torch.tensor([b]).float()
    target_slope = torch.tensor([target_slope]).float()
    sharpness1 = torch.tensor([sharpness1]).float()
    sharpness2 = torch.tensor([sharpness2]).float()

    out = torch.where(x < b, softplus_startpart(x, a, target_slope, sharpness1), softplus_endpart(x, limit, a, b, target_slope, sharpness1, sharpness2))
    return out[0].item()

class S_Curve_net(nn.Module):
    def __init__(self):
        super(S_Curve_net, self).__init__()
        self.fc1 = nn.Linear(1, 35)  
        self.fc2 = nn.Linear(35, 15)
        self.fc3 = nn.Linear(15, 5)  

    def forward(self, L):
        L = torch.tensor([L]).float().unsqueeze(0)

        out = nn.functional.softplus(self.fc1(L))
        out = nn.functional.softplus(self.fc2(out))
        out = self.fc3(out)

        a = torch.sigmoid(out[:, 0]) * 0.5 + 0.02
        b = torch.sigmoid(out[:, 2]) * 5 + a
        target_slope = torch.sigmoid(out[:, 2]) * 30 + 0.1

        sharpness1 = torch.sigmoid(out[:,3]) * 2 + 0.01
        sharpness2 = torch.sigmoid(out[:,4]) * 2 + 0.01

        return a.item(), b.item(), target_slope.item(), sharpness1.item(), sharpness2.item()
    



model_hex = "504b03040000080800000000000000000000000000000000000010001200617263686976652f646174612e706b6c46420e005a5a5a5a5a5a5a5a5a5a5a5a5a5a800263636f6c6c656374696f6e730a4f726465726564446963740a71002952710128580a0000006663312e776569676874710263746f7263682e5f7574696c730a5f72656275696c645f74656e736f725f76320a71032828580700000073746f72616765710463746f7263680a466c6f617453746f726167650a71055801000000307106580300000063707571074b23747108514b004b234b018671094b014b0186710a8968002952710b74710c52710d58080000006663312e62696173710e6803282868046805580100000031710f68074b23747110514b004b238571114b0185711289680029527113747114527115580a0000006663322e77656967687471166803282868046805580100000032711768074d0d02747118514b004b0f4b238671194b234b0186711a8968002952711b74711c52711d58080000006663322e62696173711e6803282868046805580100000033711f68074b0f747120514b004b0f8571214b0185712289680029527123747124527125580a0000006663332e77656967687471266803282868046805580100000034712768074b4b747128514b004b054b0f8671294b0f4b0186712a8968002952712b74712c52712d58080000006663332e62696173712e6803282868046805580100000035712f68074b05747130514b004b058571314b0185713289680029527133747134527135757d713658090000005f6d65746164617461713768002952713828580000000071397d713a580700000076657273696f6e713b4b01735803000000666331713c7d713d683b4b01735803000000666332713e7d713f683b4b0173580300000066633371407d7141683b4b01737573622e504b07080c4456bf6e0200006e020000504b03040000080800000000000000000000000000000000000011001300617263686976652f627974656f7264657246420f005a5a5a5a5a5a5a5a5a5a5a5a5a5a5a6c6974746c65504b0708853de3190600000006000000504b0304000008080000000000000000000000000000000000000e003e00617263686976652f646174612f3046423a005a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a2ebfbbbd164ac83ec08e4a3f7bee1e3f1c7c9fbeaaf21bbebd6fc13ed7515cbfb7d54f3fc4714a3f406a4bbf8dabb53eea42a5be620c95bd265c03c0fea23fbdb6ccd2bd74be163f46f2543f795095bd31556f3f07526cbcd941043f8cd2bdbd32ae2f3e940cda3e7595a1bf78ce3c3eb6f051bda36e19be436f593f40eca2b85acad6bfb85a4d3fdf4318bf504b0708af337ce38c0000008c000000504b0304000008080000000000000000000000000000000000000e003800617263686976652f646174612f31464234005a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a9644603f3365a8bead949bbe3ce2b83ea5afe6bec3e3bebc042aa33ee692af3fa1b4703fca0f3a3ff4b47f3dbb1a4bbf00bf823e3296e53e6ff77e3f49e347bf27df753cd25683bb170944bff99a56bfd17d81bf33822ebf4344a43e1913043f8257e33e789ce5be1206d4be1d9c323e4f0d7c3de90571beecfc6c3e63ed8bbe1ec6323f33957bbe6f5310be504b070850f5bb1e8c0000008c000000504b0304000008080000000000000000000000000000000000000e003800617263686976652f646174612f32464234005a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a0b76d53d0b40babd0e4b773da96b5d3d5ea3263e6f9bf73d3fac26be5208653e8b51a7bd2dcd21bec236173e53f5103ddbf2e73d45048b3da37fb43e52b2343ec80c523e7145a73c8970ad3db0954c3e739aa3bd2a8d213d2b435d3d150a0b3e6dea073eaac795bdaaa4243d3a54b4bdaa40af3c44598b3e5ab389bdcbfae73c4b07b03eb4fc06be9da1543e6398393e2441b1bd8f29413d5dd2cc3d8de0b03e166eb03eb659483da7b8443eb6eb703c32d8b3bbd0f6393e7bda0a3d475ecf3ee330ee3ebd2b043f1a2ba53e31eac13e9ec0143d36b001be0425113c4769e2bd1d1a983ed721903d3274d13ef1da833d99d264bc709d163ed0bd1b3e5b0ee83e0c32213ea53510bd81a8b23dd5fc513e0b4c1c3ecc6b8c3ed47061bcf28701beaa46c13cef88073ebe85b43c7f7becbd76ba5dbd2b2e27bd8b5ebdbd9a6ed4bd091ad8bdf04d77bd9301723dd5cbdebdd29d8ebe48e74ebe35e137bc2328433d5ce00a3ec5247b3ddd56f63d53d15abd2a71ccbd15d4b13b3fafaa3c3b07b23d59c74cbeae6d913dbbae1e3d212a84b97480163ea10cc9bdcfddbcbce008103e3fe7e3bb5d839c3dd90de83ddb2fd23dd2f118be9ce75b3e4772103e4be923be8d84cf3d3b597fbdb9adbbbdd160e9bad2bc3a3d50c2f93da44e91bd91fd44be390d4dbde398693ec0c19bbdecbc23bde19f0dbd4f9f70bddf84cebd5c699bbd0671783e0406fb3d921dd53db202fcbcc568f7bca56c433e05b2bc3d1098ad3d90c39ebc59355dbe4bc5493daf6cf038ba96c43d6247b5bd17119ebd2c4e53bd4f1f2c3c0f83223e1b511dbd89eed73dd219993d49af22be08ee543e709ab1bd4080923ecb4b933b01e7cebe849e133ebde74a3e10391b3ebb13c8bc6c90123e5c17b03d6fa802be634a87bd94031d3e5e66dbbc95ab96bd18caf1bbcf2906bd66cfe6bc4e25213e0225ecbb49bf3d3e0f6a88bd9ae92f3d865d8f3e7580423d0b28d63cf49623beb2838cbdb03f8c3d9756a3bc8356dc3d123bb7bdb60083bdba3299bd0da1583ee7d403beb5824c3e85c761bd2b3cb33e8cce96bc6c8e963df129253eb5ec8fbdd2e61c3eea44893d87b3373ef1740e3eb942b1bc1a7e583eb0dda23cd3f2203e0427003e7c116e3e50d3c13c52db86bdc507b43ca026783e01aa2dbb6597713e0bb7963e4bc594bdf5c34ebcbb83873df64b6b3e799c603ed49aa3bd4354ec3d2508fbba2750153dd5fb3b3d0b7000bda0dc2b3e592acf3e22722f3f5d19c33d6e6cc43e22431bbe2f3eb8bb05b3893e3ee94bbc63b96d3e6825d9bd7ffcc13e0a77b2bc08cfc73c84ff8d3d403f0b3ebea4263e2f99663e20a98cbdb03c663db18e353eaee1cebd3712613e69d177bca2f19ebc8711aa3d11281abe247d14bebb4ea43dbeaacf3c1bdaa3bbf0880d3e4086d1bd4c8a7fbed57748be91946ebe5ee34ebd1a046dbe071682bdc07322be798e693ddc642abecacace3d92914d3d60adf5bd6a85c43d842d8ebd0d85ad3d665f88bd73bdaf3b99027f3d16e330be61fc1cbdaf07363d4f663abea4870abe0d8197bc7131abbe04340cbdc97ade3dc7f4233e056e0ebde864debc78a9e1bd9b09f33d9fcb1abeb69d66bce0a711bdc8c4c0bd6df9383e393002bdf14802bd2f4bb23ef7324b3ecc21103d6e321e3eea2e9d3cd791a2bd7b5414be3401ebbcb01000bde7105e3dca3b2e3e1b755ebdcca1bfbd0d3396bcb8f85b3e05d89fbda41cd13cd65a4a3d9a42d63d630c67bd6d9f24bed399a23de0b38a3d96dddbbdcf339f3d9579d83e9676863e79160cbe60541b3ee4080dbe00e0003e26dc583e67ec35bea6e0783ee0710fbd687cd73e2407803e842cfa3d98a4043eba62f7bc3cdc1e3eb5be703db31aefbb8119c8bd81c4613d5ae3433e455e01bec035843e44df90bd3315213ec9bcac3e2bd0583df17722bd3e6c893e86a521bee9ce233e598d823e2b5e953ccb88df3cb41122bc3a8ec0bda96b203b2b4fd0bdca74e0bdd09ed93d67eac53d0edb79bdbb8ad33cba7b213ef8bf633ecb64083fa8ec753e737d573ef4b5c9bb10bbb8bb2eb7f1bdc928633ddcc8683e07a00cbe967a3bbc2deb333eb2a271bd6b6f3e3e59766d3d4b333d3eef1e1bbe632034bd5147283e8f4c403d36a71f3e951e453e4695c9bc0baa06be54f11a3e95d0e7bd95e7e23c62df4e3d488603bdbb58643e48580f3eb40f2a3e78c3283d7ad2b9bd8df57cbc7a029ebd31353ebe9e80d1bddaffe0bd8e93133d635c22bee7379abdb8c8673d859742bbcfe3943da1d73fbe945f83bc379de2bd291e3fbd37a34bbca02007bde78d063e3ece5d3d6c6c783d11fdb139ebddd33cea10f03d605c493e113dd83d7ac5fc3db6d3e23c7650543e3d6ef33d1d11f33de9ce1c3ed2f30abe73bcdc3ab169603e1135ae3db6eb6b3e575f2c3eaa189dbe03ffc33d8053943eb0a8413d00d9b5bdf338b63bfa4fe6bc3c591f3e289d543a2d13bd3d6f0917beae30a0bd59e87f3df606e03dd6fe1bbcc426a93e4fa319bef61f48bca03dc6bd7c26e0bcc7bc743d9321e33df1f8d13dea4c043e6d59f4bdf217c23d2bece33c893c12bee17a7f3e2fd092bdd808e23b74fd26bc1cb7dfbd3df1a53dc4a6a03d8b51e7beb3c495bdda46eb3d61440cbee2a801bdcdc853bd45aa0dbe6787353e662c31bd5045a4bde7d633be0e621f3c564119bd9300a9bddfa953bd8672783cc4b9193e769b95bd230847bd7c3a143ea13c713d131b2a3e18e50cbeaebaeebdeb1aaabc8f5dc53e24548e3ef8f056bdf4f6723e7182da3dac15023dfa337b3e1358af3d85ced93e747c893e4db4063fd740173d1740ed3ce73a1d3e2a59dabd25b46abcf46562bcca95d03dc46b17be6b86a73d4609053e9852803daca7aa3d9e5783bdb836ba3d39dcb73d8327eabdc21a3e3e8034af3e777b08bed346033e504b07087ce68c3f3408000034080000504b0304000008080000000000000000000000000000000000000e001000617263686976652f646174612f3346420c005a5a5a5a5a5a5a5a5a5a5a5aef5f3c3ebea9673e32a8153dd8650dbe6453ebbcefca7e3e7198213e5dd2ffbd0d05503eb370303d95c70f3eb31404bed006b5bc980d2cbd63a2d33d504b07087940f45a3c0000003c000000504b0304000008080000000000000000000000000000000000000e000800617263686976652f646174612f34464204005a5a5a5a871b35bf9991c6be2667753e29a2debed708c8bec2fd96becc6276bfab9c7a3d3eb77c3e6d5adbbe1f79963c065686bd46c945bf08698bbeb225dabefc5e673df78b45bd173819bd0c7e55be2a90103ba7e39e3d60b23c3e2b6a07bdfb2cff3dab7d6c3e2d6c403e4564123eca5fcb3d74fa673d1c154abe301a88bec4c073be4331333e50ef5f3e836cef3dbc1faebed9ce2ebe15d34dbcf0fa31be0f2940be814488bed9e16a3e2a2e5d3e87f0163e85bc01be9641c03ff4c80cbe4acd3d3dd4b61f3f6ab0433fb60a483e2a4cde3f3458953abe9ae6bd353c9a3ecd1a103dd3dc39be261d7f3fd6f7793f826c633f7251223fb3e1ab3f5404983f1f914d3ff74a5f3fbef48e3f1d20393f71b03b3fc3458e3fe0f96d3fe801843f477c8a3f63144b3f30643b3f65d85f3f504b07083c3094852c0100002c010000504b0304000008080000000000000000000000000000000000000e001800617263686976652f646174612f35464214005a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5aa3f769be30743e3ee41a8bbeecc21abe5bd62e3f504b07089bf74f771400000014000000504b0304000008080000000000000000000000000000000000000f002f00617263686976652f76657273696f6e46422b005a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a330a504b0708d19e67550200000002000000504b0304000008080000000000000000000000000000000000001e003200617263686976652f2e646174612f73657269616c697a6174696f6e5f696446422e005a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a30353334333237383234393439373936353734373033393338353030383432313739313230303433504b07082024e56b2800000028000000504b01020000000008080000000000000c4456bf6e0200006e020000100000000000000000000000000000000000617263686976652f646174612e706b6c504b0102000000000808000000000000853de31906000000060000001100000000000000000000000000be020000617263686976652f627974656f72646572504b0102000000000808000000000000af337ce38c0000008c0000000e0000000000000000000000000016030000617263686976652f646174612f30504b010200000000080800000000000050f5bb1e8c0000008c0000000e000000000000000000000000001c040000617263686976652f646174612f31504b01020000000008080000000000007ce68c3f34080000340800000e000000000000000000000000001c050000617263686976652f646174612f32504b01020000000008080000000000007940f45a3c0000003c0000000e00000000000000000000000000c40d0000617263686976652f646174612f33504b01020000000008080000000000003c3094852c0100002c0100000e000000000000000000000000004c0e0000617263686976652f646174612f34504b01020000000008080000000000009bf74f7714000000140000000e00000000000000000000000000bc0f0000617263686976652f646174612f35504b0102000000000808000000000000d19e675502000000020000000f0000000000000000000000000024100000617263686976652f76657273696f6e504b01020000000008080000000000002024e56b28000000280000001e0000000000000000000000000092100000617263686976652f2e646174612f73657269616c697a6174696f6e5f6964504b06062c000000000000001e032d0000000000000000000a000000000000000a000000000000006e020000000000003811000000000000504b060700000000a61300000000000001000000504b0506000000000a000a006e020000381100000000"
model_bytes = bytes.fromhex(model_hex)
buffer = io.BytesIO(model_bytes)
model_state_dict = torch.load(buffer, weights_only=True)