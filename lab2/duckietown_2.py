from gym_duckietown.tasks.task_solution import TaskSolution
import numpy as np
import cv2


class DontCrushDuckieTaskSolution(TaskSolution):
    def __init__(self, generated_task):
        super().__init__(generated_task)

    def laneChange(self, env, lane):
        if (lane == 'left'):
            turn = 1
        if (lane == 'right'):
            turn = -1
        img, _, _, _ = env.step([1, turn * 30])
        for i in range(10):
            img, _, _, _ = env.step([1, 0])
            env.render()
        img, _, _, _ = env.step([1, -turn * 30])

    def obstacleAvoidanceCheck(self, img, lowRange, upperRange):
        img = np.ascontiguousarray(img)
        rgbImage = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        mask = cv2.inRange(rgbImage, lowRange, upperRange)
        count = cv2.countNonZero(mask)
        percent = count / (img.size - count)
        return percent

    def solve(self):
        env = self.generated_task['env']
        img, _, _, _ = env.step([0, 0])
        pixelPercentage = 0.013
        lowYellow = np.array([0, 150, 150])
        upperYellow = np.array([150, 255, 255])

        condition = True
        changedLane = False
        while condition:
            img, _, _, _ = env.step([1, 0])

            percent = self.obstacleAvoidanceCheck(img, lowYellow, upperYellow)

            if not changedLane and percent > pixelPercentage:
                changedLane = True
                self.laneChange(env, 'left')

            if changedLane and percent < pixelPercentage:
                changedLane = False
                self.laneChange(env, 'right')

            condition = True
            env.render()
