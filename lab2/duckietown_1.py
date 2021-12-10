from gym_duckietown.tasks.task_solution import TaskSolution
import numpy as np
import cv2


class DontCrushDuckieTaskSolution(TaskSolution):
    def __init__(self, generated_task):
        super().__init__(generated_task)

    def solve(self):
        env = self.generated_task['env']
        img, _, _, _ = env.step([0, 0])
        lowYellow = np.array([0, 150, 150])
        upperYellow = np.array([150, 255, 255])

        condition = True
        while condition:
            img, _, _, _ = env.step([1, 0])
            
            img = np.ascontiguousarray(img)
            rgbImage = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

            maskYellow = cv2.inRange(rgbImage, lowYellow, upperYellow)
            count = cv2.countNonZero(maskYellow)

            percent = count / (img.size - count)
            condition = percent < 0.025
            env.render()
