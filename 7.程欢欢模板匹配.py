import time, sensor, image
from image import SEARCH_EX, SEARCH_DS

sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QQVGA)
sensor.set_contrast(3)

num_quantity = 8
num_model = [image.Image(str(n) + ".pgm") for n in range(1, num_quantity + 1)]

clock = time.clock()
img_colorful = sensor.alloc_extra_fb(160, 120, sensor.RGB565)
img_to_matching = sensor.alloc_extra_fb(35, 45, sensor.GRAYSCALE)
threshold = (0, 70)
scale = 1

while True:
    clock.tick()
    img = sensor.snapshot()
    img_colorful.draw_image(img, 0, 0)
    blobs = img.find_blobs([threshold])
    
    if blobs:
        for blob in blobs:
            if blob.pixels() > 50 and 100 > blob.h() > 10 and blob.w() > 3:
                scale = 40 / blob.h()
                img_to_matching.draw_image(img, 0, 0, roi=(blob.x() - 2, blob.y() - 2, blob.w() + 4, blob.h() + 4), x_scale=scale, y_scale=scale)
                for n in range(num_quantity):
                    try:
                        r = img_to_matching.find_template(num_model[n], threshold=0.7, roi=(0,0,160,120),step=2, search=image.SEARCH_EX)
                        if r:
                            img_colorful.draw_rectangle(blob[:4], color=(255, 0, 0))
                            img_colorful.draw_string(blob[0], blob[1], str(n + 1), scale=2, color=(255, 0, 0))
                    except Exception as e:
                        print("An error occurred:", e)
    
    img_colorful.draw_string(0, 0, "FPS:%.2f" % clock.fps(), scale=2)
