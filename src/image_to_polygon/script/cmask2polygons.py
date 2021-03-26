import cv2
import numpy as np
import PIL.ImageDraw as ImageDraw
import PIL.Image as Image


__all__ = ['get_polygons_per_class']


def _get_unique_colors(img_arr):
    return np.unique(img_arr.reshape(-1, 3), axis=0)


def _get_cls_from_color_mask(color_mask, cls_color_map):
    cls_color_map_rev = {str(color): cls_name for cls_name, color in cls_color_map.items()}
    colors = _get_unique_colors(img_arr=color_mask)

    classes = list()
    for color in colors:
        color = str(tuple(color))  # Convert color type to str for hashing
        cls = cls_color_map_rev[color]
        classes.append(cls)
    return classes


def _get_bin_mask(color_mask, cls_name, cls_color_map):    
    color = cls_color_map[cls_name]
    cls_pixels = np.all(color_mask==color, axis=-1)
    
    height, width, _ = color_mask.shape
    bin_mask = np.zeros([height, width])
    bin_mask[cls_pixels] = 255
    bin_mask[~cls_pixels] = 0
    return bin_mask.astype(np.uint8)


def _get_polygons_from_bin_mask(bin_mask, min_area, epsilon_param, pt_type, add_closept):
    contours, _ = cv2.findContours(bin_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_TC89_KCOS)  # contours, hierarchy

    contour_approxed_list = list()
    for contour in contours:
        if cv2.contourArea(contour) < min_area: continue
        epsilon = epsilon_param * cv2.arcLength(curve=contour, closed=True)
        contour_approxed = cv2.approxPolyDP(curve=contour, epsilon=epsilon, closed=True)

        # Convert data type of contours for serializing (np.ndarray --> list, np.int64 --> pt_type)
        contour_approxed_converted = list()
        for xy in contour_approxed:
            xy = tuple(map(pt_type, xy[0]))
            contour_approxed_converted.append(xy)

        # Append end point for representing closed
        if add_closept:
            contour_approxed_converted.append(contour_approxed_converted[0])

        contour_approxed_converted = tuple(contour_approxed_converted)

        contour_approxed_list.append(contour_approxed_converted)
    
    return contour_approxed_list


def get_polygons_per_class(color_mask, cls_color_map, min_area=100.0, epsilon_param=8e-4, pt_type=int, add_closept=False):
    classes = _get_cls_from_color_mask(color_mask=color_mask, cls_color_map=cls_color_map)

    polygons_per_class = dict()
    for cls in classes:
        # print(cls)
        bin_mask = _get_bin_mask(color_mask=color_mask, cls_name=cls, cls_color_map=cls_color_map)
        polygons = _get_polygons_from_bin_mask(bin_mask=bin_mask, min_area=min_area, epsilon_param=epsilon_param, pt_type=pt_type, add_closept=add_closept)
        polygons_per_class[cls] = polygons
        # print(polygons_per_class)
    
    # print(polygons_per_class["Building"])

    # *************************************************************

    image = Image.new("RGB", (1920, 1080))
    draw = ImageDraw.Draw(image)

    for points in polygons_per_class["Building"]:
        draw.polygon((points), fill=(128,0,0))
    for points in polygons_per_class["Road Marking"]:
        draw.polygon((points), fill=(0,128,0))
    for points in polygons_per_class["Guard Rail"]:
        draw.polygon((points), fill=(0,0,128))

    # *************************************************************
    # image.show()
    
    return polygons_per_class, image
