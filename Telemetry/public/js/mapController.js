let map, boatPath, latLines = [], lngLines = [], gridCenters = [], gridCenterPoints = [], debug = false;

// const attitash = { lat: 42.8489, lng: -70.9829 };
// const startPathCoords = [{ lat: 42.849810669147935, lng: -70.98818573987138 }];
const quinsigamond = { lat: 42.276268, lng: -71.756629 };
const startPathCoords = { lat: 42.276268, lng: -71.7576294 };
const gridCorners = {topleft: { lat: 42.278568, lng: -71.757519 }, botleft: { lat: 42.278568, lng: -71.755000 }, botright: { lat: 42.274568, lng: -71.755000 }, topright: { lat: 42.278568, lng: -71.755119 }};

const boatSVG = {
    anchor: new google.maps.Point(200, 0),
    path: "M186.771 14.593 C 69.712 144.111,10.495 260.896,1.632 379.716 C 1.113 386.673,0.815 469.496,0.813 606.694 L 0.811 822.718 200.425 822.718 L 400.039 822.718 399.782 598.580 C 399.508 360.108,399.687 371.991,395.966 345.639 C 381.460 242.897,332.445 145.393,247.094 49.493 C 237.349 38.544,200.549 0.793,199.660 0.833 C 199.401 0.844,193.601 7.036,186.771 14.593 M219.254 69.986 C 298.936 158.616,345.383 247.655,360.672 341.084 C 365.511 370.652,365.353 366.198,365.687 482.556 L 365.995 589.858 200.442 589.858 L 34.888 589.858 34.897 489.858 C 34.902 429.782,35.231 385.971,35.722 380.122 C 44.327 277.595,94.918 173.447,189.731 63.071 L 200.719 50.281 203.874 53.335 C 205.610 55.015,212.531 62.508,219.254 69.986 M365.923 706.694 L 365.923 789.452 200.406 789.452 L 34.888 789.452 34.888 706.694 L 34.888 623.935 200.406 623.935 L 365.923 623.935 365.923 706.694",
    strokeColor: "#FFF",
    fillOpacity: 1,
    scale: 0.025,
};
const waypoint = {
        path: 'M -2,0 a 2,2 0 1,0 4,0 a 2,2 0 1,0 -4,0',
        scale: 3,
        strokeColor: '#004d00',
        fillColor: '#00e600',
        fillOpacity: .5,
};


// generate GridLines and their centers
const genGridlines = () => {
    const squareRad = 0.001; // 10 meters is 4th decimal places I think? (near the equator it is 11)

    for (let i = gridCorners.topleft.lng; i <= gridCorners.topright.lng; i += squareRad) {
        latLines.push(new google.maps.Polyline({
            path: [{lat: gridCorners.topleft.lat, lng: i}, {lat: gridCorners.botleft.lat, lng: i}],
        }));
    }

    for (let i = gridCorners.topleft.lat; i >= gridCorners.botleft.lat; i -= squareRad) {
        lngLines.push( new google.maps.Polyline({
            path: [{lat: i, lng:  gridCorners.topleft.lng}, {lat: i, lng:  gridCorners.topright.lng}],
        }));
    }

    let gridIntersects = [];

    // generating all the points where the 2 gridlines intersect
    latLines.forEach((latLine, i) => {
        let temp = [];
        lngLines.forEach((lngLine, j) => {
            // new google.maps.Marker({
            //     position: { lat: lngLine.getPath().getAt(0).lat(), lng: latLine.getPath().getAt(0).lng() },
            //     icon: waypoint,
            //     map,
            //     label: i.toString() + j.toString(),
            // });
            temp.push({x: i, y: j, lat: lngLine.getPath().getAt(0).lat(), lng: latLine.getPath().getAt(0).lng()});
            // temp.push({x: i, y: j, lat: lngLine[0].lat, lng: latLine[0].lng});
        });
        gridIntersects.push(temp);
    });

    for (let i = 0; i < gridIntersects.length - 1; i++){
        let temp = []
        leftLngLine = gridIntersects[i]
        rightLngLine = gridIntersects[i + 1]

        for (let j = 0; j < leftLngLine.length - 1; j++){
            let topLeftCorner = leftLngLine[j]
            let topRightCorner = rightLngLine[j]
            let botLeftCorner = leftLngLine[j + 1]
            let center = { lat: (topLeftCorner.lat + botLeftCorner.lat)/2,lng: (topLeftCorner.lng + topRightCorner.lng)/2 };
            
            gridCenterPoints.push(new google.maps.Marker({
                position: center,
                icon: waypoint,
                label: i.toString() + j.toString(),
            }));

            temp.push({x: i, y: j, lat: center.lat, lng: center.lng});
        }
        gridCenters.push(temp);
    }
}


const toggleDebug = () => {
    console.log('debug toggled');
    if (!debug) {
        debug = true;
        gridCenterPoints.forEach(center => {console.log(center);center.setMap(map);});
        latLines.forEach(line => line.setMap(map));
        lngLines.forEach(line => line.setMap(map));
    } else {
        debug = false;
        gridCenterPoints.forEach(center => center.setMap(null));
        latLines.forEach(line => line.setMap(null));
        lngLines.forEach(line => line.setMap(null));
    }
}


const initMap = () => {
    // The map, centered at quinsigamond
    map = new google.maps.Map(document.getElementById('mapCanvas'), {
        zoom: 17,
        center: quinsigamond,
        mapTypeId: 'terrain',
        // mapTypeId: 'satellite',
        // tilt: 60,
    });
    genGridlines();

    boatPath = new google.maps.Polyline({
        path: startPathCoords,
        icons: [
                {
                    icon: {
                        path: "M 0,-1 0,1",
                        strokeColor: "#CC33FF",
                        strokeOpacity: 1,
                        scale: 4,
                    },
                    offset: "0",
                    repeat: "20px"
                },
                {
                    icon: {
                        path: "M -2,0 0,-2 2,0 0,2 z",
                        strokeColor: "#F00",
                        fillColor: "#F00",
                        fillOpacity: 1,
                      },
                    offset: "0%"
                },
                {
                    icon: boatSVG,
                    offset: "100%",
                }],
        geodesic: true,
        strokeOpacity: 0,
        map,
    });

    map.addListener("click", (event) => {
        boatPath.getPath().push(event.latLng);
        console.log('{ lat: '+ event.latLng.lat() +', lng: '+ event.latLng.lng() +' }');
        // mock.push('{ lat: '+ event.latLng.lat() +', lng: '+ event.latLng.lng() +' }');
    });

    // waypoint
    new google.maps.Marker({
        position: { lat: 42.276268, lng: -71.755619 },
        icon: waypoint,
        map,
        title: 'first',
    });

    new google.maps.Marker({
        position: { lat: 42.276568, lng: -71.756619 },
        icon: waypoint,
        map,
        title: 'second',
    });

    new google.maps.Marker({
        position: { lat: 42.275858, lng: -71.756629 },
        icon: waypoint,
        map,
        title: 'third',
    });
};