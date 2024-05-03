import cv2 
import numpy as np
# cap = cv2.VideoCapture("E:\coding\computervision\ABU\WIN_20231206_13_41_02_Pro.mp4") # Green
# cap = cv2.VideoCapture("C:\\Users\\User\\Pictures\\Camera Roll\\WIN_20240114_16_30_55_Pro.mp4") # yellow3
# cap = cv2.VideoCapture("C:\\Users\\User\\Pictures\\Camera Roll\\WIN_20240114_16_20_05_Pro.mp4") # yellow2
# cap = cv2.VideoCapture("C:\\Users\\User\\Pictures\\Camera Roll\\WIN_20240212_16_08_55_Pro.mp4") # green stupid cam
# cap = cv2.VideoCapture("C:\\Users\\User\\Pictures\\Camera Roll\\WIN_20240212_16_12_38_Pro.mp4") # yellow stupid cam

# cap = cv2.VideoCapture(1)

def vertical(y_pos,binary_image):
    apos = 999
    # Get the height and width of the image
    height, width = binary_image.shape
    # Extract a single column from the binary image
    binary_image = binary_image[:, y_pos:y_pos+1]
    binary_image_index = np.where(np.array(binary_image[:, 0]) == 0)
    
    # Extract 8 sections from the binary image using NumPy
    section_height = height // 8
    binary_image_8bit = []

    for i in range(8):
        section = binary_image[i * section_height:(i + 1) * section_height, :]
        binary_value = 1 if np.any(section == 0) else 0
        binary_image_8bit.append(binary_value)

    # Calculate the average position of white pixels in the column
    if len(binary_image_index[0]) > 0:
        apos = np.average(binary_image_index[0])

        # Draw a circle at the average position
        cv2.circle(binary_image, (y_pos, np.round(apos).astype(np.int32)), 5, (255, 255, 255), -1)

    horizontal_output = {"outputText":np.array2string(np.round(apos).astype(np.int32), separator='') + ':' + ''.join(map(str, binary_image_8bit)),
                          "img":binary_image}

    return horizontal_output

def horizontal(x_pos,binrary_image):
    apos = 999
    # Extract a single row from the binary image
    binary_image = binrary_image[x_pos:x_pos+1, :]
    binary_image_index = np.where(np.array(binary_image[0]) == 0)
    height, width = binrary_image.shape
    # Extract 8 sections from the binary image using NumPy
    section_width = width // 8
    binary_image_8bit = []

    for i in range(8):
        section = binary_image[:, i * section_width:(i + 1) * section_width]
        binary_value = 1 if np.any(section == 0) else  0
        binary_image_8bit.append(binary_value)

    # Calculate the average position of white pixels in the row
    if len(binary_image_index[0]) > 0:
        apos = np.average(binary_image_index[0])
    horizontal_output = {"outputText":np.array2string(np.round(apos).astype(np.int32),separator='') +':'+ ''.join(map(str, binary_image_8bit)),
                         "img":binary_image,
                         "Error":150 - np.round(apos).astype(np.int32)}

    return horizontal_output

def BGR2BIN(cap,threshold = 80,height = 300, width = 300):

    check , frame = cap.read() 
    frames = cv2.resize(frame,(height,width))
    frame = cv2.GaussianBlur(frames,(23,23),cv2.BORDER_DEFAULT)
    frame = cv2.cvtColor(frame , cv2.COLOR_RGB2HSV)
    # height, width, _ = frame.shape
    
    h, s, v = cv2.split(frame)
    thresh,th1 = cv2.threshold(s,threshold,255,cv2.THRESH_BINARY_INV)
    showLine = th1.copy()
    output_BGR2BIN = {"RGB":frames, "showLine" : showLine, "th1":th1, "Height":height, "width":width, "check":check}
    return output_BGR2BIN

def visualize_Line(visual_frame, visual_text):
     x1 = 100
     x2 = 200
     y1 = 100
     y2 = 200
     camline = {
         "x1": 80,
         "x2": 360,
         "y1": 160,
         "y2": 480
     } 
     # Draw line
     cv2.line(visual_frame, (0, x1), (width, x1), (255, 255, 255), 1)
     cv2.line(visual_frame, (0, x2), (width, x2), (255, 255, 255), 1)
     cv2.line(visual_frame, (y1, 0), (y1, height), (255, 255, 255), 1)
     cv2.line(visual_frame, (y2, 0), (y2, height), (255, 255, 255), 1) 
     # Label line
     cv2.putText(
         visual_text,
         "line1",
         (100,100),
         cv2.FONT_HERSHEY_SIMPLEX,
         0.5,
         (255, 255, 255),
         1,
         cv2.LINE_AA,
     )
     cv2.putText(
         visual_text,
         "line2",
         (100,200),
         cv2.FONT_HERSHEY_SIMPLEX,
         0.5,
         (255, 255, 255),
         1,
         cv2.LINE_AA,
     )
     cv2.putText(
         visual_text,
         "line3",
         (200,100),
         cv2.FONT_HERSHEY_SIMPLEX,
         0.5,
         (255, 255, 255),
         1,
         cv2.LINE_AA,
     )
     cv2.putText(
         visual_text,
         "line4",
         (200,200),
         cv2.FONT_HERSHEY_SIMPLEX,
         0.5,
         (255, 255, 255),
         1,
         cv2.LINE_AA,
     )

#cap 50 300 300
def linetrackVariable(cap, threshold, h, w):
    global check, height, width, show_frames, show_Line, show_Horizon, show_ver_Left, show_ver_Right, info_ver_Left, info_ver_Right, info_Horizon
    output_BGR2BIN = BGR2BIN(cap, threshold = 50, height = 300, width = 300)

    check = output_BGR2BIN["check"]
    height = output_BGR2BIN["Height"]
    width = output_BGR2BIN["width"]
    th1 = output_BGR2BIN["th1"]

    output_Horizon = horizontal(100, th1)
    output_vertical_Left = vertical(100, th1)
    output_vertical_Right = vertical(200, th1)
    
    show_frames = output_BGR2BIN["RGB"]
    show_Line = output_BGR2BIN["showLine"]
    show_Horizon = output_Horizon["img"]
    show_ver_Left = output_vertical_Left["img"]
    show_ver_Right = output_vertical_Right["img"]

    info_ver_Left = output_vertical_Left["outputText"]
    info_ver_Right = output_vertical_Right["outputText"]
    info_Horizon = output_Horizon["outputText"]

    error = output_Horizon["Error"]
    return error

def opencam(cap):
    output_BGR2BIN = BGR2BIN(cap, threshold = 50, height = 300, width = 300)
    output = output_BGR2BIN["RGB"]
    if check == True :
        return output
    else:
        return "Sad"

# while (cap.isOpened()):
#     output_BGR2BIN = BGR2BIN(cap, threshold = 50, height = 300, width = 300)

#     check = output_BGR2BIN["check"]
#     height = output_BGR2BIN["Height"]
#     width = output_BGR2BIN["width"]
#     th1 = output_BGR2BIN["th1"]

#     output_Horizon = horizontal(100, th1)
#     output_vertical_Left = vertical(100, th1)
#     output_vertical_Right = vertical(200, th1)
    
#     show_frames = output_BGR2BIN["RGB"]
#     show_Line = output_BGR2BIN["showLine"]
#     show_Horizon = output_Horizon["img"]
#     show_ver_Left = output_vertical_Left["img"]
#     show_ver_Right = output_vertical_Right["img"]

#     info_ver_Left = output_vertical_Left["outputText"]
#     info_ver_Right = output_vertical_Right["outputText"]
#     info_Horizon = output_Horizon["outputText"]

#     error = output_Horizon["Error"]

#     if check == True :
#         visualize_Line(show_Line, show_frames)
        
#         # kernel = np.ones((8,8),np.uint8)
#         # th1 = cv2.erode(th1,kernel,iterations=2)
#         # th1 = cv2.morphologyEx(th1,cv2.MORPH_OPEN,kernel,iterations=4)

#         cv2.imshow("original",show_frames)
#         cv2.imshow("Bin",show_Line)
#         cv2.imshow("Horizon",show_Horizon)
#         cv2.imshow("Left",show_ver_Left)
#         cv2.imshow("Right",show_ver_Right)

#         print(f"Left : {info_ver_Left} -------------- Right : {info_ver_Right}")
#         print(f"Line Tracking : {info_Horizon}")
#         print(f"Error : {error}")

#         if cv2.waitKey(30) & 0xFF == ord("q"):
#             break
#     else :
#         break

# cap.release()
# cv2.destroyAllWindows()