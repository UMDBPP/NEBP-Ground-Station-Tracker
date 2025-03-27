OVERLAY_LAYERS['Flights'] = {};

/* retrieve flight for a single launch as a GeoJSON FeatureCollection */
async function getFlightLayer(flight_number, filetype="geojson") {
    if (filetype == "geojson"){
        /* asynchronously load launch from GeoJSON file */
        return L.geoJson.ajax(DATA_DIRECTORY + 'flights/ns' + flight_number + '.geojson', {
            'onEachFeature': popupFeaturePropertiesOnClick
        });
    } else if(filetype == "kml"){
        /* asynchronously load launch from KML file */
        // return L.geoJson.ajax(DATA_DIRECTORY + 'flights/ns' + flight_number + '.KML', {
        //     middleware: function(data){
        //         console.log(data);
        //         return toGeoJSON.kml(data);
        //     },
        //     'onEachFeature': popupFeaturePropertiesOnClick
        // });

        /* synchronously load launch from KML file */
        let kml_layer;
        await $.ajax(DATA_DIRECTORY + 'flights/ns' + flight_number + '.KML').done(function(xml) {
            kml_layer = L.geoJson(toGeoJSON.kml(xml), {
                'onEachFeature': popupFeaturePropertiesOnClick
            });
        });
        return kml_layer;
    }
    
}

/* remove all flight layers from the map */
function removeFlightLayers() {
    for (let layer_group in OVERLAY_LAYERS) {
        if (layer_group === 'Flights') {
            for (let layer_name in OVERLAY_LAYERS[layer_group]) {
                LAYER_CONTROL.removeLayer(OVERLAY_LAYERS[layer_group][layer_name]);
                MAP.removeLayer(OVERLAY_LAYERS[layer_group][layer_name]);
                delete OVERLAY_LAYERS[layer_group][layer_name];
            }
        }
    }
}

/* deselect all flight layers from the map, but keep them in the layer control */
function hideFlightLayers() {
    for (let layer_group in OVERLAY_LAYERS) {
        if (layer_group === 'Flights') {
            for (let layer_name in OVERLAY_LAYERS[layer_group]) {
                MAP.removeLayer(OVERLAY_LAYERS[layer_group][layer_name]);
            }
        }
    }
}

/* refresh map with new flights */
async function updateFlightLayers(resize = false) {
    ACTIVE_FLIGHT_LAYERS = LAYER_CONTROL.getActiveOverlayLayers()['Flights'];
    let layer_index = 0;
    let flight_layer, flight_number;

    removeFlightLayers();

    await fetch(DATA_DIRECTORY + 'flights/flight_list.json')
        .then((response) => response.json())
        .then(async (flight_list) => {
            // let flight_list_len = flight_list['flights'].length;
            for (let flight of flight_list['flights'].reverse()){ // Reverse order here so new flights are at top of list
                if(typeof(flight['number']) == 'number'){
                    flight_number = String(flight['number']).padStart(3, '0');
                } else{
                    flight_number = String(flight['number']);
                }
                if(flight["filetype"] != null){
                    /* Get layer from geojson */
                    flight_layer = await getFlightLayer(flight_number, flight["filetype"]);
                } else{
                    /* Get layer from geojson */
                    flight_layer = await getFlightLayer(flight_number);
                }
                /* Add layer to overlays */
                OVERLAY_LAYERS['Flights']['NS' + flight_number] = flight_layer;
                LAYER_CONTROL.addOverlay(flight_layer, 'NS-' + flight_number, 'Flights');
                
                // if (ACTIVE_FLIGHT_LAYERS != null) {
                //     /* add predict layers to map if they were already selected previously */
                //     if (ACTIVE_FLIGHT_LAYERS['NS-' + flight_number] != null) {
                //         MAP.addLayer(flight_layer);
                //     }
                // } else {
                //     /* if no layers were selected previously, add the first few layers */
                //     if (layer_index < 3) {
                //         MAP.addLayer(flight_layer);
                //     }

                //     // /* if no layers were selected previously, add the last few layers */
                //     // if ((flight_list_len - layer_index) <= 3) {
                //     //     MAP.addLayer(flight_layer);
                //     // }
    
                //     layer_index++;
                // }
            }
        });

    /* TypeError: this._northeast is undefined */
    if (resize) {
        resizeToOverlayLayers();
    }
}

function downloadFlightsKML() {
    let active_flight_layers = LAYER_CONTROL.getActiveOverlayLayers()['Flights'];

    if (Object.keys(active_flight_layers).length > 0) {
        for (let launch_location_name in active_flight_layers) {
            let flight_geojson = active_flight_layers[launch_location_name].toGeoJSON();

            if (flight_geojson['features'].length > 0) {
                let output_kml = tokml(flight_geojson);

                output_kml = output_kml.replace(/<LineString>/g, '<LineString><extrude>1</extrude><tesselate>1</tesselate><altitudeMode>absolute</altitudeMode>');

                let download_filename = 'predicts_' + launch_location_name.replace(/[-_:ZT ]/g, '') + '.kml';
                let download_link = document.createElement('a');
                let xml_blob = new Blob([output_kml], {'type': 'text/xml'});

                download_link.setAttribute('href', window.URL.createObjectURL(xml_blob));
                download_link.setAttribute('download', download_filename);

                download_link.click();
            } else {
                alert('Flights have not loaded yet.');
            }
        }
    } else {
        alert('No Flights.');
    }
}

function downloadURI(uri, name) {
    let download_link = document.createElement('a');
    download_link.download = name;
    download_link.href = uri;
    download_link.click();
}
