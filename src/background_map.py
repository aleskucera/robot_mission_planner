import requests
from PIL import Image
from geopy.distance import geodesic


URL_PREF = "https://maps.geoapify.com/v1/staticmap?style=osm-carto"
API_KEY = "8f3be3c0c8484eceb15b0f50218c8c02"

    
def get_url(w, h, corners):
    '''
    Get url for the background map.

    Parameters:
    -----------
    w : int
        Width of the image.
    h : int
        Height of the image.
    corners : list
        Corners of the area to get the map of.

    Returns:
    --------
    url : str
        Url for the background map.
    '''
    url = URL_PREF
    w_url = f'&width={w}'
    h_url = f'&height={h}'
    area_url = f'&area=rect:{corners[0]},{corners[1]},{corners[2]},{corners[3]}'
    api_url = f'&apiKey={API_KEY}'
    
    url = url + w_url + h_url + area_url + api_url

    return url

def get_background_image(min_long, max_long, min_lat, max_lat, x_margin, y_margin):
    '''
    Get background image of the area.

    Parameters:
    -----------
    min_long : float
        Minimum longitude.
    max_long : float
        Maximum longitude.
    min_lat : float
        Minimum latitude.
    max_lat : float
        Maximum latitude.
    x_margin : float
        Margin in x direction.
    y_margin : float
        Margin in y direction.

    Returns:
    --------
    bg_map : PIL.Image
        Background map.
    '''
    width_m = geodesic((max_lat + y_margin, max_long + x_margin), (min_lat - y_margin, max_long + x_margin))
    height_m = geodesic((max_lat + y_margin, max_long + x_margin), (max_lat + y_margin, min_long - x_margin))
    ratio = width_m / height_m
    
    size_limit = min(4000, 150*min(width_m.meters, height_m.meters))
    width = int(size_limit)
    height = int(width * ratio)

    while width > size_limit or height > size_limit:  # anything more can lead to wrongly cut background image (maybe only with large ratios)
        width = int(width * 0.9)
        height = int(width * ratio)

    corners = [min_long - x_margin, max_lat + y_margin, max_long + x_margin, min_lat - y_margin]
    url = get_url(width, height, corners)

    bg_map = Image.open(requests.get(url, stream=True).raw)

    return bg_map
