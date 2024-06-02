class Environemnt:
    def __init__(self, x, y):
        self.left_lower_ball = (x + 3, y + 3)
        self.middle_lower_ball = (x + 7, y + 3)
        self.right_lower_ball = (x + 11, y + 3)
        self.left_central_ball = (x + 3, y + 7)
        self.middle_central_ball = (x + 7, y + 7)
        self.right_central_ball = (x + 11, y + 7)
        self.left_upper_ball = (x + 3, y + 11)
        self.middle_upper_ball = (x + 7, x + 11)
        self.right_upper_ball = (x + 11, x + 11)
        self.barrel = (x + 7, y + 27)
