# #!/usr/bin/env python3
# import cv2


# def main():
#     for i in range(50):
#         cap = cv2.VideoCapture(i)
#         if cap.isOpened():
#             print("port:", "/dev/video"+str(i))
#             while True:
#                 ret, frame = cap.read()
#                 cv2.imshow("/dev/video"+str(i), frame)
#                 if cv2.waitKey(1) & 0xFF == ord('q'):
#                     break
#     # ros_operator = RosOperator()
#     # if ros_operator.init_camera():
#     #     print("camera opened")
#     #     ros_operator.run()
#     # else:
#     #     print("camera error")

# if __name__ == '__main__':
#     main() 

# !/usr/bin/env python3
import cv2

def main():
    device_cnt = 0
    for i in range(50):
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            print("port:", "/dev/video" + str(i))
            while True:
                ret, frame = cap.read()
                if not ret:
                    print("Failed to grab frame")
                    break
                device_cnt += 1 
                cv2.imshow("/dev/video" + str(i), frame)
                
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):  # 按 'q' 键退出程序
                    print("Exiting...")
                    cap.release()
                    cv2.destroyAllWindows()
                    return
                elif key == 32:  # 按空格键进入下一个摄像头
                    print("Switching to next camera...")
                    break
            
            cap.release()
            cv2.destroyAllWindows()
        else:
            print("Camera not opened: /dev/video" + str(i))
    print(device_cnt)

if __name__ == '__main__':
    main()


