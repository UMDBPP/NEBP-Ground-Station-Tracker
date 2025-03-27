if (typeof DATA_DIRECTORY === 'undefined') {
    var DATA_DIRECTORY = 'assets/data/';
}

let BASE_LAYERS = {
    'Esri Topography': L.tileLayer.provider('Esri.WorldTopoMap'),
    'Esri Road': L.tileLayer.provider('Esri.WorldStreetMap'),
    'Esri Gray': L.tileLayer.provider('Esri.WorldGrayCanvas'),
    'Esri Imagery': L.tileLayer.provider('Esri.WorldImagery'),
    // 'maritime chart': L.tileLayer('https://tileservice.charts.noaa.gov/tiles/50000_1/{z}/{x}/{y}.png', {transparent: true})
};

// BASE_LAYERS['maritime chart'].on('add', function (event) {
//     event.target._mapToAdd.addLayer(BASE_LAYERS['Esri Gray']);
// });

/* asynchronously load polygons of controlled airspace from GeoJSON file */
let CONTROLLED_AIRSPACE_LAYER = L.geoJson.ajax(DATA_DIRECTORY + 'reference/controlled_airspace.geojson', {
    'onEachFeature': popupFeaturePropertiesOnClick, 'style': function (feature) {
        switch (feature.properties['LOCAL_TYPE']) {
            case 'R':
                return {'color': '#ea2027'};
            case 'CLASS_B':
                return {'color': '#0652dd'};
            case 'CLASS_C':
                return {'color': '#6f1e51'};
            case 'CLASS_D':
                return {'color': '#0652dd', 'dashArray': '4'};
            default:
                return {'color': '#6f1e51', 'dashArray': '4'};
        }
    }, 'attribution': 'Airspace - FAA'
});

/* asynchronously load polygons of uncontrolled airspace from GeoJSON file */
let UNCONTROLLED_AIRSPACE_LAYER = L.geoJson.ajax(DATA_DIRECTORY + 'reference/uncontrolled_airspace.geojson', {
    'onEachFeature': popupFeaturePropertiesOnClick, 'style': function (feature) {
        return {'color': '#6f1e51', 'dashArray': '4'};
    }, 'attribution': 'Airspace &copy; FAA'
});

/* asynchronously load launch locations from GeoJSON file */
// let LAUNCH_LOCATIONS_LAYER = L.geoJson.ajax(DATA_DIRECTORY + 'reference/launch_locations.geojson', {
//     'onEachFeature': popupFeaturePropertiesOnClick
// });

/* asynchronously load McDonald's locations from GeoJSON file */
// let MCDONALDS_LOCATIONS_LAYER = L.geoJson.ajax(DATA_DIRECTORY + 'mcdonalds_locations.geojson', {
//     'onEachFeature': popupFeaturePropertiesOnClick, 'pointToLayer': function (feature, latlng) {
//         return L.circleMarker(latlng, {
//             'radius': 4, 'fillColor': '#ee5a24', 'color': '#000', 'weight': 1, 'opacity': 1, 'fillOpacity': 0.8
//         });
//     }
// });

/* load public schools layer from ArcGIS Online */
// let PUBLIC_SCHOOLS_LAYER = L.esri.featureLayer({
//     url: 'https://services1.arcgis.com/Ua5sjt3LWTPigjyD/arcgis/rest/services/Public_School_Location_201819/FeatureServer/0',
//     'onEachFeature': popupFeaturePropertiesOnClick,
//     'pointToLayer': function (feature, latlng) {
//         return L.circleMarker(latlng, {
//             'radius': 4, 'fillColor': '#24ee2e', 'color': '#000', 'weight': 1, 'opacity': 1, 'fillOpacity': 0.8
//         });
//     }
// });

/* dictionary to contain toggleable layers */
let OVERLAY_LAYERS = {
    'Reference': {
        'Controlled Airspace': CONTROLLED_AIRSPACE_LAYER, 'Uncontrolled Airspace': UNCONTROLLED_AIRSPACE_LAYER, 
    }
};

/* add Leaflet map to 'map' div with grouped layer control */
let MAP = L.map('map', {
    'layers': [BASE_LAYERS['Esri Topography']], 'zoomSnap': 0, 'zoomControl': false, 'touchZoom': true, 'attributionControl': false, dragging: true
});
MAP.on('layeradd', sinkReferenceLayers);
MAP.addControl(L.control.scale());



// OVERLAY_LAYERS['reference']['McDonald\'s Locations'] = MCDONALDS_LOCATIONS_LAYER;
// OVERLAY_LAYERS['Reference']['Launch Locations'] = LAUNCH_LOCATIONS_LAYER;
// OVERLAY_LAYERS['Reference']['Public Schools'] = PUBLIC_SCHOOLS_LAYER;

let LAYER_CONTROL = L.control.groupedLayers(BASE_LAYERS, OVERLAY_LAYERS);
MAP.addControl(LAYER_CONTROL);