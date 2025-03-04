class Graph:
    edges = [
        {3: (3, 330)},
        {5: (3, 320)},
        {6: (3, 330)},
        {0: (1, 330), 4: (0, 720), 7: (3, 850)},
        {3: (2, 720), 5: (0, 320), 16: (3, 280)},
        {1: (1, 320), 4: (2, 320), 6: (0, 1050)},
        {2: (1, 330), 5: (2, 1050), 10: (3, 850)},
        {3: (1, 850), 8: (0, 1020), 12: (3, 770)},
        {7: (2, 1020), 9: (0, 350), 11: (3, 380)},
        {8: (2, 350), 10: (0, 720), 17: (1, 220)},
        {6: (1, 850), 9: (2, 720), 15: (3, 770)},
        {8: (1, 380), 13: (3, 390), 18: (0, 255)},
        {7: (1, 770), 13: (0, 1020)},
        {11: (1, 390), 12: (2, 1020), 14: (0, 420)},
        {13: (2, 420), 15: (0, 650), 19: (1, 230)},
        {10: (1, 770), 14: (2, 650)},
        {4: (1, 280)},
        {9: (3, 220)},
        {11: (0, 255)},
        {14: (3, 230)},
    ]

    paths = {(1,16): [5,4],
            (16,0): [4,3],
            (16,2): [4,5,6],
            (0,17): [3,7,8,9],
            (2,17): [6,10,9],
            (0,18): [3,7,8,11],
            (2,18): [6,10,9,8,11],
            (0,19): [3,7,12,13,14],
            (2,19): [6,10,15,14],
            (17,18): [9,8,11],
            (17,19): [9,8,11,13,14],
            (18,19): [11,13,14]
    }