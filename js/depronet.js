var DIR = 'img/depronet';

var nodes = null;
var edges = null;

// Called when the Visualization API is loaded.
function draw() {

    var options = {
        groups: {
            devm: {
                shape: 'icon',
                icon: {
                    face: 'FontAwesome',
                    code: '\uf0ad',
                    size: 50,
                    color: '#4e9f51'
                }
            },
            devf: {
                shape: 'icon',
                icon: {
                    face: 'FontAwesome',
                    code: '\uf0ad',
                    size: 50,
                    color: '#d16e81'
                }
            },
            desm: {
                shape: 'icon',
                icon: {
                    face: 'FontAwesome',
                    code: '\uf1fc',
                    size: 50,
                    color: '#4e9f51'
                }
            },
            desf: {
                shape: 'icon',
                icon: {
                    face: 'FontAwesome',
                    code: '\uf1fc',
                    size: 50,
                    color: '#d16e81'
                }
            },
            money: {
                shape: 'icon',
                icon: {
                    face: 'FontAwesome',
                    code: '\uf159',
                    size: 50,
                    color: '#c28ade'
                }
            }
        },
        nodes: {
            borderWidth: 0,
            size: 30,
            color: {
                border: 'lightgray',
                background: '#ffffff'
            },
            font: {
                face: 'myFont',
                color: '#807f7f',
                size: 40
            },
            shapeProperties: {
                useBorderWithImage: true
            }
        },
        edges: {
            color: 'lightgray'
        }
    };
    // create people.
    // value corresponds with the age of the person
    var DIR = '../img/depronet/';
    nodes = [
        /*{
                        id: 1,
                        shape: 'circularImage',
                        image: DIR + 'img1-0.jpg',
                        imagehigh: 'himg1-0.jpg'
                    }, {
                        id: 2,
                        shape: 'circularImage',
                        image: DIR + 'img1-1.jpg',
                        imagehigh: 'himg1-1.jpg'
                    }, {
                        id: 3,
                        shape: 'circularImage',
                        image: DIR + 'img1-2.jpg',
                        imagehigh: 'himg1-2.jpg'
                    }, {
                        id: 4,
                        shape: 'circularImage',
                        image: DIR + 'img1-3.jpg',
                        imagehigh: 'himg1-3.jpg'
                    }, {
                        id: 5,
                        shape: 'circularImage',
                        image: DIR + 'img1-4.jpg',
                        imagehigh: 'himg1-4.jpg'
                    }, {
                        id: 6,
                        shape: 'circularImage',
                        image: DIR + 'img1-5.jpg',
                        imagehigh: 'himg1-5.jpg'
                    }, {
                        id: 7,
                        shape: 'circularImage',
                        image: DIR + 'img1-6.jpg',
                        imagehigh: 'himg1-6.jpg'
                    }, {
                        id: 8,
                        shape: 'circularImage',
                        image: DIR + 'img1-7.jpg',
                        imagehigh: 'himg1-7.jpg'
                    }, */
        {
            id: 1,
            shape: 'circularImage',
            size: 70,
            image: DIR + 'logo.png',
            imagehigh: 'logo.png',
            borderWidth: 10
    }, {
            id: 2,
            label: '손은주',
            group: 'devf'
    }, {
            id: 3,
            label: '장선혁',
            group: 'devm'
    }, {
            id: 4,
            label: '김예진',
            group: 'devf'
    }, {
            id: 5,
            label: '손규빈',
            group: 'devm'
    }, {
            id: 6,
            label: '권윤환',
            group: 'devm'
    }, {
            id: 7,
            label: '남시욱',
            group: 'devm'
    }, {
            id: 8,
            label: '변우진',
            group: 'devm'
    }, {
            id: 9,
            label: '김종욱',
            group: 'devm'
    }, {
            id: 10,
            shape: 'circularImage',
            size: 50,
            image: DIR + 'pres.jpg',
            imagehigh: 'hpres.jpg',
            label: '이지명',
    }, {
            id: 11,
            label: '이건희',
            group: 'devm'
    }, {
            id: 12,
            label: '이솔',
            group: 'desf'
    }, {
            id: 13,
            label: '정재연',
            group: 'desf'
    }, {
            id: 14,
            label: '박세진',
            group: 'desf'
    }, {
            id: 15,
            label: '고현진',
            group: 'desf'
    }, {
            id: 16,
            label: '서상민',
            group: 'desf'
    }, {
            id: 17,
            label: '이주원',
            group: 'desf'
    }, {
            id: 18,
            label: '이담윤',
            group: 'desf'
    }, {
            id: 19,
            label: '김다찬',
            group: 'desm'
    }, {
            id: 20,
            label: '김호영',
            group: 'desf'
    }, {
            id: 21,
            label: '조미현',
            group: 'desf'
    }, {
            id: 22,
            label: '송경아',
            group: 'desf'
    }, {
            id: 23,
            label: '민경원',
            group: 'desf'
    }, {
            id: 24,
            label: '전병준',
            group: 'money'
    }, {
            id: 25,
            label: 'Depropedia',
            shape: 'icon',
            icon: {
                face: 'FontAwesome',
                code: '\uf02d',
                size: 50,
                color: '#faad44'
            }
    }, {
            id: 26,
            label: 'Mirror Mirror',
            shape: 'icon',
            icon: {
                face: 'FontAwesome',
                code: '\uf047',
                size: 50,
                color: '#faad44'
            }
    }, {
            id: 27,
            label: 'Facebook Link',
            shape: 'icon',
            icon: {
                face: 'FontAwesome',
                code: '\uf09a',
                size: 50,
                color: '#faad44'
            }
    }, {
            id: 28,
            label: 'Interactive Web',
            shape: 'icon',
            icon: {
                face: 'FontAwesome',
                code: '\uf1b3',
                size: 50,
                color: '#faad44'
            }
    }, {
            id: 29,
            label: 'Food Commander',
            shape: 'icon',
            icon: {
                face: 'FontAwesome',
                code: '\uf0f5',
                size: 50,
                color: '#faad44'
            }
    }, {
            id: 30,
            label: 'Homepage',
            shape: 'icon',
            icon: {
                face: 'FontAwesome',
                code: '\uf015',
                size: 50,
                color: '#faad44'
            }
    }, {
            id: 31,
            shape: 'image',
            image: DIR + 'pictures.png',
            imagehigh: 'hpictures.png'
    }];

    // create connections between people
    // value corresponds with the amount of contact between two people
    edges = [
        /*{
                        from: 1,
                        to: 2
                    }, {
                        from: 39,
                        to: 1
                    }, {
                        from: 2,
                        to: 3
                    }, {
                        from: 39,
                        to: 4
                    }, {
                        from: 39,
                        to: 5
                    }, {
                        from: 5,
                        to: 6
                    }, {
                        from: 6,
                        to: 7
                    }, {
                        from: 6,
                        to: 8
                    }, */
        {
            from: 1,
            to: 31
    }, {
            from: 1,
            to: 25
    }, {
            from: 1,
            to: 26
    }, {
            from: 1,
            to: 27
    }, {
            from: 1,
            to: 28
    }, {
            from: 1,
            to: 29
    }, {
            from: 1,
            to: 30
    }, { // Depropedia
            from: 25,
            to: 3
    }, {
            from: 25,
            to: 16
    }, { // Mirror Mirror
            from: 26,
            to: 3
    }, {
            from: 26,
            to: 10
    }, {
            from: 26,
            to: 12
    }, {
            from: 26,
            to: 16
    }, {
            from: 26,
            to: 18
    }, { // Facebook Link
            from: 27,
            to: 5
    }, {
            from: 27,
            to: 6
    }, {
            from: 27,
            to: 8
    }, {
            from: 27,
            to: 10
    }, {
            from: 27,
            to: 17
    }, {
            from: 27,
            to: 20
    }, { // Interactive Web
            from: 28,
            to: 3
    }, {
            from: 28,
            to: 9
    }, {
            from: 28,
            to: 16
    }, {
            from: 28,
            to: 14
    }, { // Food commander
            from: 29,
            to: 2
    }, {
            from: 29,
            to: 22
    }, {
            from: 29,
            to: 15
    }, {
            from: 29,
            to: 13
    }, {
            from: 29,
            to: 21
    }, { // Homepage
            from: 30,
            to: 7
    }, {
            from: 30,
            to: 11
    }, {
            from: 30,
            to: 19
    }, {
            from: 30,
            to: 4
    }, {
            from: 30,
            to: 23
    }, { // special
            from: 10,
            to: 24,
    }, {
            from: 24,
            to: 10,
    }];
    // create a network
    var container = document.getElementById('mynetwork');
    var data = {
        nodes: nodes,
        edges: edges
    };

    var network = new vis.Network(container, data, options);
    network.on("doubleClick", function (params) {
        if (typeof (nodes[params.nodes - 1]["image"]) !== 'undefined') {
            var s = DIR + nodes[params.nodes - 1]["imagehigh"];
            $('.imagepreview').attr('src', s);
            $('#imagemodal').modal('show');
        } else if (params.nodes == 24) {
            $('#imagemodal2').modal('show');
            var elem = document.getElementById("vid");
            if (elem.requestFullscreen) {
                elem.requestFullscreen();
            } else if (elem.msRequestFullscreen) {
                elem.msRequestFullscreen();
            } else if (elem.mozRequestFullScreen) {
                elem.mozRequestFullScreen();
            } else if (elem.webkitRequestFullscreen) {
                elem.webkitRequestFullscreen();
            }
            elem.play();

        }
    });

}