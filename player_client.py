import cv2
import time
import socket
import numpy as np
import mediapipe as mp

mp_face_mesh = mp.solutions.face_mesh
face_mesh = mp_face_mesh.FaceMesh(
    min_detection_confidence=0.8,
    min_tracking_confidence=0.8,
    max_num_faces=1,
)   
ERROR = 4
MAX_ANGLE_THRESH = 15
steering_angle = 0
status_panel = np.zeros((94, 640, 3), dtype=np.uint8)
prev_frame_time, current_frame_time, fps = 0, 0, 0


def calc_fps(prev_frame_del_time, curr_frame_del_time):
    return int(1 / (curr_frame_del_time - prev_frame_del_time))

def send_steering_signal(host='localhost', port=5555, signal=0):
    try:
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_socket.connect((host, port))
        client_socket.send(str(signal).encode())
        client_socket.close()
        print(f"Client just sent a steering signal: {str(signal)} to the simulator server")
        print()
    except ConnectionRefusedError:
        print(f"Client was unable to send a steering signal to the simulator server [Connection Refused]")
        print()

def app(resolution=(640, 480), cam_src_id=0):
    global steering_angle, prev_frame_time, current_frame_time, fps
    
    cam_src = cv2.VideoCapture(cam_src_id)

    while True:
        ret, frame = cam_src.read()
        if not ret:
            continue
        
        current_frame_time = time.time()
        fps = calc_fps(prev_frame_time, current_frame_time)
        prev_frame_time = current_frame_time

        frame = cv2.resize(frame, (resolution[0], resolution[1] - 94))
        height, width, _ = frame.shape
        frame = cv2.flip(frame, 1)
        image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        image.flags.writeable = False
        results = face_mesh.process(image)
        image.flags.writeable = True

        face_2d = []
        face_3d = []

        if results.multi_face_landmarks:
            for face_landmarks in results.multi_face_landmarks:
                for idx, lm in enumerate(face_landmarks.landmark):
                    if idx in (1, 33, 61, 199, 263, 291):
                        if idx == 1:
                            nose_2d = (lm.x * width, lm.y * height)
                            nose_3d = (lm.x * width, lm.y * height, lm.z * 8000)

                    x, y = int(lm.x * width), int(lm.y * height)
                    
                    face_2d.append([x, y])
                    face_3d.append([x, y, lm.z])
                
                face_2d = np.array(face_2d, dtype=np.float64)
                face_3d = np.array(face_3d, dtype=np.float64)

                # camera matrix
                focal_length = 1 * width
                cam_matrix = np.array([
                    [focal_length, 0, height / 2],
                    [0, focal_length, width / 2],
                    [0, 0, 1]
                ])

                # distance matrix
                dist_matrix = np.zeros((4, 1), dtype=np.float64)

                # solve PnP
                _, rot_vec, trans_vec = cv2.solvePnP(face_3d, face_2d, cam_matrix, dist_matrix)

                # convert rotational vector into a rotational matrix
                rot_matrix, _ = cv2.Rodrigues(rot_vec)

                # get angles/
                angles, _, _, _, _, _ = cv2.RQDecomp3x3(rot_matrix)

                # get the y rotational degree
                y, x = angles[0] * 360, angles[1] * 360
                x = min(x, MAX_ANGLE_THRESH)
                x = max(x, -MAX_ANGLE_THRESH)
                steering_angle = x / MAX_ANGLE_THRESH

                if x < -ERROR:
                    head_pose = "Left"
                    driving_direct = "Left"
                elif x > ERROR:
                    head_pose = "Right"
                    driving_direct = "Right"
                elif y < -ERROR:
                    head_pose = "Down"
                    steering_angle = 0
                    driving_direct = "Front"
                else:
                    head_pose = "Forward"
                    steering_angle = 0
                    driving_direct = "Front"

                send_steering_signal(signal=steering_angle)

                # nose direction
                nose_3d_projection, _ = cv2.projectPoints(nose_3d, rot_vec, trans_vec, cam_matrix, dist_matrix)
                p1 = (int(nose_2d[0]), int(nose_2d[1]))
                p2 = (int(nose_3d_projection[0][0][0]), int(nose_3d_projection[0][0][1]))

                cv2.line(frame, p1, p2, (0, 0, 255), 2)
                
                frame = np.concatenate((status_panel, frame), axis=0)
                cv2.putText(frame, f"Head Pose: {head_pose}", (20, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv2.putText(frame, f"Driving Direct: {driving_direct}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv2.putText(frame, f"Steering Angle: {round(steering_angle, 2)}", (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv2.putText(frame, f"FPS: {fps}", (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv2.imshow("HEAD POSE BASED GAME CONTROLLER", frame)
        
        if cv2.waitKey(1) in (27, ord('q'), ord('Q')):
            break

    cam_src.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    app()
    