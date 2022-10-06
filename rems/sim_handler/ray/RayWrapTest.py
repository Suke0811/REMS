from rems.sim_handler.ray import RayWrapper

class SayHello:
    def __init__(self, words, val):
        self.words = words
        self.val = val

    def say_once(self):
        print(self.words)

    def say_n_times(self, n):
        [print(self.words) for i in range(n)]

    def more_words(self, w):
        self.new_words = w


import ray
ray.init()
r = RayWrapper(SayHello('hi', 5))

r.say_once()
print('next')
r.say_n_times(5)
