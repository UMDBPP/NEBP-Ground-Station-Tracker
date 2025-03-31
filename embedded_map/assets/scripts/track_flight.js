OVERLAY_LAYERS["Flight"] = {}

var ground_station_marker_geojson = {
    "type": "Feature",
    "properties": {
        "name": "Ground Station",
        "altitude_m": 0
    },
    "geometry": {
        "type": "Point",
        "coordinates": [0, 0, 0]
    }
};
OVERLAY_LAYERS["Flight"]["Ground Station"] = ground_station_marker_geojson;


/* remove all flight layers from the map */
function removeFlightLayers() {
    for (let layer_group in OVERLAY_LAYERS) {
        if (layer_group === 'Flight') {
            for (let layer_name in OVERLAY_LAYERS[layer_group]) {
                LAYER_CONTROL.removeLayer(OVERLAY_LAYERS[layer_group][layer_name]);
                MAP.removeLayer(OVERLAY_LAYERS[layer_group][layer_name]);
                delete OVERLAY_LAYERS[layer_group][layer_name];
            }
        }
    }
}


function setGroundStationLocation(lat, lon, alt){
    // Remove previous ground station marker and balloon ground track
    removeFlightLayers()
    
    // Update position in ground station marker geojson template
    ground_station_marker_geojson["geometry"]["coordinates"] = [lon, lat, alt]
    ground_station_marker_geojson["properties"]["altitude_m"] = alt

    // Add ground station marker to map
    OVERLAY_LAYERS["Flight"]["Ground Station"] = L.geoJson(ground_station_marker_geojson, {
        'onEachFeature': popupFeaturePropertiesOnClick
    });
    LAYER_CONTROL.addOverlay(OVERLAY_LAYERS["Flight"]["Ground Station"], 'Ground Station', 'Flight');
    MAP.addLayer(OVERLAY_LAYERS["Flight"]["Ground Station"]);
}


function addBalloonPosition(timestamp, lat, lon, alt, service_type, comment){
    // Create geojson object for the new balloon position
    balloon_marker_geojson = {
        "type": "Feature",
        "properties": {
            "name": "Balloon Position",
            "time": timestamp,
            "altitude_m": alt,
            "service":service_type,
            "comment":comment
        },
        "geometry": {
            "type": "Point",
            "coordinates": [lon, lat, alt]
        }
    }

    // Add balloon position marker to map
    OVERLAY_LAYERS["Flight"]["Balloon"] = L.geoJson(balloon_marker_geojson, {
        'onEachFeature': popupFeaturePropertiesOnClick
    });
    LAYER_CONTROL.addOverlay(OVERLAY_LAYERS["Flight"]["Balloon"], 'Balloon', 'Flight');
    MAP.addLayer(OVERLAY_LAYERS["Flight"]["Balloon"]);
}
