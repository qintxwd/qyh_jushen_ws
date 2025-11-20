import os
import glob
import yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion
from qyh_vr_calibration_msgs.msg import VRActionSample, VRCalibrationProfile
from qyh_vr_calibration_msgs.srv import (
    ListProfiles,
    GetProfile,
    UpdateActionSample,
    DeleteActionSamples,
    DeleteProfile
)

class VRCalibrationNode(Node):
    def __init__(self):
        super().__init__('vr_calibration_node')
        
        # Define storage path: ~/qyh_jushen_ws/persistent/vr_calibration
        self.storage_dir = os.path.join(
            os.path.expanduser("~"),
            "qyh_jushen_ws",
            "persistent",
            "vr_calibration"
        )
        if not os.path.exists(self.storage_dir):
            os.makedirs(self.storage_dir)
            self.get_logger().info(f"Created storage directory: {self.storage_dir}")
        else:
            self.get_logger().info(f"Using storage directory: {self.storage_dir}")

        # Create services
        self.srv_list = self.create_service(
            ListProfiles, 'vr_calibration/list_profiles', self.list_profiles_callback)
        self.srv_get = self.create_service(
            GetProfile, 'vr_calibration/get_profile', self.get_profile_callback)
        self.srv_update = self.create_service(
            UpdateActionSample, 'vr_calibration/update_action_sample', self.update_action_sample_callback)
        self.srv_delete_samples = self.create_service(
            DeleteActionSamples, 'vr_calibration/delete_action_samples', self.delete_action_samples_callback)
        self.srv_delete_profile = self.create_service(
            DeleteProfile, 'vr_calibration/delete_profile', self.delete_profile_callback)

        self.get_logger().info("VR Calibration Node started.")

    def _get_file_path(self, username):
        # Sanitize username to prevent path traversal
        safe_username = "".join([c for c in username if c.isalnum() or c in ('-', '_')])
        return os.path.join(self.storage_dir, f"{safe_username}.yaml")

    def _pose_to_dict(self, pose_msg):
        return {
            'position': {
                'x': pose_msg.position.x,
                'y': pose_msg.position.y,
                'z': pose_msg.position.z
            },
            'orientation': {
                'x': pose_msg.orientation.x,
                'y': pose_msg.orientation.y,
                'z': pose_msg.orientation.z,
                'w': pose_msg.orientation.w
            }
        }

    def _dict_to_pose(self, data):
        p = Pose()
        p.position.x = float(data['position']['x'])
        p.position.y = float(data['position']['y'])
        p.position.z = float(data['position']['z'])
        p.orientation.x = float(data['orientation']['x'])
        p.orientation.y = float(data['orientation']['y'])
        p.orientation.z = float(data['orientation']['z'])
        p.orientation.w = float(data['orientation']['w'])
        return p

    def _sample_to_dict(self, sample_msg):
        return {
            'sample_index': sample_msg.sample_index,
            'pose': self._pose_to_dict(sample_msg.pose)
        }

    def _dict_to_sample(self, data):
        s = VRActionSample()
        s.sample_index = data['sample_index']
        s.pose = self._dict_to_pose(data['pose'])
        return s

    def _load_profile_data(self, username):
        file_path = self._get_file_path(username)
        if not os.path.exists(file_path):
            return None
        try:
            with open(file_path, 'r') as f:
                return yaml.safe_load(f)
        except Exception as e:
            self.get_logger().error(f"Failed to load profile for {username}: {e}")
            return None

    def _save_profile_data(self, username, data):
        file_path = self._get_file_path(username)
        try:
            with open(file_path, 'w') as f:
                yaml.dump(data, f)
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to save profile for {username}: {e}")
            return False

    def _create_profile_msg(self, username, data):
        msg = VRCalibrationProfile()
        msg.username = username
        if data and 'samples' in data:
            for s_data in data['samples']:
                msg.samples.append(self._dict_to_sample(s_data))
        return msg

    def list_profiles_callback(self, request, response):
        response.profiles = []
        try:
            files = glob.glob(os.path.join(self.storage_dir, "*.yaml"))
            for f in files:
                username = os.path.splitext(os.path.basename(f))[0]
                data = self._load_profile_data(username)
                if data:
                    response.profiles.append(self._create_profile_msg(username, data))
            response.success = True
            response.message = f"Found {len(response.profiles)} profiles."
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def get_profile_callback(self, request, response):
        data = self._load_profile_data(request.username)
        if data:
            response.found = True
            response.profile = self._create_profile_msg(request.username, data)
            response.message = "Profile found."
        else:
            response.found = False
            response.message = "Profile not found."
        return response

    def update_action_sample_callback(self, request, response):
        username = request.username
        new_sample = request.sample
        
        data = self._load_profile_data(username)
        if data is None:
            data = {'username': username, 'samples': []}
        
        # Check if sample_index exists
        updated = False
        for i, s in enumerate(data['samples']):
            if s['sample_index'] == new_sample.sample_index:
                data['samples'][i] = self._sample_to_dict(new_sample)
                updated = True
                break
        
        if not updated:
            data['samples'].append(self._sample_to_dict(new_sample))
            # Optional: sort by index
            data['samples'].sort(key=lambda x: x['sample_index'])

        if self._save_profile_data(username, data):
            response.success = True
            response.profile = self._create_profile_msg(username, data)
            response.message = "Sample updated successfully."
        else:
            response.success = False
            response.message = "Failed to save profile."
        
        return response

    def delete_action_samples_callback(self, request, response):
        username = request.username
        sample_index = request.sample_index
        
        data = self._load_profile_data(username)
        if data is None:
            response.success = False
            response.message = "Profile not found."
            return response

        original_count = len(data['samples'])
        if sample_index == -1:
            # Delete all samples
            data['samples'] = []
            msg = "All samples deleted."
        else:
            # Delete specific sample
            data['samples'] = [s for s in data['samples'] if s['sample_index'] != sample_index]
            msg = f"Sample {sample_index} deleted."

        if len(data['samples']) == original_count and sample_index != -1:
             response.success = False
             response.message = f"Sample {sample_index} not found."
             return response

        if self._save_profile_data(username, data):
            response.success = True
            response.profile = self._create_profile_msg(username, data)
            response.message = msg
        else:
            response.success = False
            response.message = "Failed to save profile."
        
        return response

    def delete_profile_callback(self, request, response):
        file_path = self._get_file_path(request.username)
        if os.path.exists(file_path):
            try:
                os.remove(file_path)
                response.success = True
                response.message = f"Profile for {request.username} deleted."
            except Exception as e:
                response.success = False
                response.message = f"Failed to delete file: {e}"
        else:
            response.success = False
            response.message = "Profile not found."
        return response

def main(args=None):
    rclpy.init(args=args)
    node = VRCalibrationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
