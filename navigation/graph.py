from array import array

class Graph:
    edges = [
        {3: bytearray([3, 330])},
        {5: bytearray([3, 320])},
        {6: bytearray([3, 330])},
        {0: bytearray([1, 330]), 4: bytearray([0, 720]), 7: bytearray([3, 850])},
        {3: bytearray([2, 720]), 5: bytearray([0, 320]), 16: bytearray([3, 280])},
        {1: bytearray([1, 320]), 4: bytearray([2, 320]), 6: bytearray([0, 1050])},
        {2: bytearray([1, 330]), 5: bytearray([2, 1050]), 10: bytearray([3, 850])},
        {3: bytearray([1, 850]), 8: bytearray([0, 1020]), 12: bytearray([3, 770])},
        {7: bytearray([2, 1020]), 9: bytearray([0, 350]), 11: bytearray([3, 380])},
        {8: bytearray([2, 350]), 10: bytearray([0, 720]), 17: bytearray([1, 220])},
        {6: bytearray([1, 850]), 9: bytearray([2, 720]), 15: bytearray([3, 770])},
        {8: bytearray([1, 380]), 13: bytearray([3, 390]), 18: bytearray([2, 255])},
        {7: bytearray([1, 770]), 13: bytearray([0, 1020])},
        {11: bytearray([1, 390]), 12: bytearray([2, 1020]), 14: bytearray([0, 420])},
        {13: bytearray([2, 420]), 15: bytearray([0, 650]), 19: bytearray([1, 230])},
        {10: bytearray([1, 770]), 14: bytearray([2, 650])},
        {4: bytearray([1, 280])},
        {9: bytearray([3, 220])},
        {11: bytearray([0, 255])},
        {14: bytearray([3, 230])},
    ]

    paths = {
        0: {
            1: array('I',[0,3,4,5,1]),
            16: array('I',[0,3,4,16]),
            17: array('I',[0,3,7,8,9,17]),
            18: array('I',[0,3,7,8,11,18]),
            19: array('I',[0,3,7,12,13,14,19]),
        },
        1: {
            1: array('I',[1,1]),
            16: array('I',[1,5,4,16]),
        },
        2: {
            1: array('I',[2,6,5,1]),
            16: array('I',[2,6,5,4,16]),
            17: array('I',[2,6,10,9,17]),
            18: array('I',[2,6,10,9,8,11,18]),
            19: array('I',[2,6,10,15,14,19]),
        },
        16: {
            0: array('I',[16,4,3,0]),
            2: array('I',[16,4,5,6,2]),
        },
        17: {
            0: array('I',[17,9,8,7,3,0]),
            2: array('I',[17,9,10,6,2]),
        },
        18: {
            0: array('I',[18,11,8,7,3,0]),
            2: array('I',[18,11,8,9,10,6,2]),
        },
        19: {
            0: array('I',[19,14,13,12,7,3,0]),
            2: array('I',[19,14,15,10,6,2]),
        },
    }