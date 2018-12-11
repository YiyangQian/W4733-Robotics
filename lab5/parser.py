import rospy, cv2, cv_bridge, numpy

def parse(image_path, target_path):
    image = cv2.imread(image_path)
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    lower_red = numpy.array([0, 43, 46])
    upper_red = numpy.array([20, 255, 250])
    red_mask = cv2.inRange(hsv, lower_red, upper_red)
    
    h, w, d = image.shape

    search_top = 5*h/8-5
    search_bot = 7*h/8 - 5

    search_left = w/4
    search_right = w*3/4

    red_mask_crop = red_mask[search_top:search_bot,search_left:search_right]
    image_crop = hsv[search_top:search_bot,search_left:search_right]
    masked = cv2.bitwise_and(image_crop, image_crop, mask=red_mask_crop)
    print(red_mask_crop.shape)

    cv2.imshow('before_crop',red_mask)
    cv2.imshow('crop',red_mask_crop)
    cv2.waitKey(0)

    cv2.imwrite(target_path, red_mask_crop)
    rgb_masked =  cv2.cvtColor(masked, cv2.COLOR_HSV2BGR)
    cv2.imwrite('./template/template_left_rgb_masked.jpg', rgb_masked)

# parse('./template/left.jpg', './template/template_left_mask.jpg')
#parse('./template/saved_35.jpg', './template/template_right_mask.jpg')
#parse('./template/star.jpg', './template/template_star_mask.jpg')

def match(original_path, template_path):
    original_img = cv2.imread(original_path)
    template_img = cv2.imread(template_path)
    #print(type(original_img), type(template_img))
    res = cv2.matchTemplate(original_img, template_img, cv2.TM_CCOEFF_NORMED)
    print(numpy.max(res))

def match_mask(original_path, template_path):
    original_img = cv2.imread(original_path)
    template_img = cv2.imread(template_path)
    hsv = cv2.cvtColor(original_img, cv2.COLOR_BGR2HSV)
    hsv_template = cv2.cvtColor(template_img, cv2.COLOR_BGR2HSV)
    lower_red = numpy.array([0, 43, 46])
    upper_red = numpy.array([20, 255, 250])
    red_mask = cv2.inRange(hsv, lower_red, upper_red)
    red_mask_template = cv2.inRange(hsv_template, lower_red, upper_red)

    res = cv2.matchTemplate(red_mask, red_mask_template, cv2.TM_CCOEFF_NORMED)
    print(numpy.max(res))

match_mask("./template/left.jpg", './template/template_left_rgb_masked.jpg')
#match_mask("./template/left.jpg", './template/template_left_masked.jpg')
