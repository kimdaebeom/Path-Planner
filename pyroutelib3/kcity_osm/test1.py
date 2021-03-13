import pandas as pd
from pyroutelib3 import Router # Import the router
router = Router("car", "A2_LINK.osm") # Initialise it

start = router.findNode(37.2388759, 126.7729287)
end = router.findNode(37.2404968, 126.7741559)

status, route = router.doRoute(start, end) # Find the route - a list of OSM nodes
data = pd.DataFrame(columns=['latitude','longitude'])
if status == 'success':
    routeLatLons = list(map(router.nodeLatLon, route)) # Get actual route coordinates
    for latlon in routeLatLons:
        print(str(latlon)[1:-1])
        ind = str(latlon).find(',')
        data = data.append({'latitude':str(latlon)[1:ind-1],'longitude':str(latlon)[ind+1:-1]}, ignore_index=True)

data.to_csv('test.csv')        

