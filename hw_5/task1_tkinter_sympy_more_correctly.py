from tkinter import *
import math
from time import time

import heapq
from shapely.geometry import Polygon


class PriorityQueue:
    def __init__(self):
        self.elements = []

    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item.get_position(), item))

    def get(self):
        elem = heapq.heappop(self.elements)
        return elem[2]

    def empty(self):
        return len(self.elements) == 0


class StepPath:
    def __init__(self, x, y, yaw, parent, len_path):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.parent = parent
        self.hash = self.get_hash()
        self.len_path = len_path

    def get_position(self):
        return self.x, self.y, self.yaw

    def get_parent(self):
        return self.parent

    def get_hash(self):
        str_state = ' '.join([str(round(self.x, 0)), str(round(self.y, 0)), str(round(self.yaw, 1))])
        return str_state


class AStar:
    def __init__(self, start, target, obstacles, root, canvas):
        self.root = root
        self.canvas = canvas

        self.pq = PriorityQueue()

        self.start = start
        self.target = target
        self.obstacles = obstacles
        self.cant_move_from = set()
        self.visited_field = set()

        self.pq.put(self.start, self.heuristic_distance(self.start, self.target))

        self.step_len = 20
        self.step_angle = 10
        self.max_angle = 40
        self.const_angle = 1e-8

        self.angles = [angle/180*math.pi+self.const_angle for angle in range(-self.max_angle, self.max_angle+self.step_angle, self.step_angle)]

    def generate_steps(self, position, back):
        next_steps = []
        len_path = position.len_path + 1
        steps = [-self.step_len, self.step_len] if back else [self.step_len]
        for angle in self.angles:
            for step_len in steps:
                new_position = StepPath(*self.make_move(position.get_position(), (step_len, angle)), position, len_path)
                if new_position.hash not in self.visited_field and new_position.hash not in self.cant_move_from:
                    if self.check_collides(new_position, self.obstacles):
                        self.visited_field.add(new_position.hash)
                        next_steps.append(new_position)
                    else:
                        self.cant_move_from.add(new_position.hash)
        return next_steps

    def make_move(self, position, step):
        x, y, yaw = position
        step_len, angle_step = step
        sign_angle = 1 if angle_step >= 0 else -1

        dx = math.sin(angle_step+yaw) * step_len
        dy = math.cos(angle_step+yaw) * step_len

        step_dist = math.sqrt(dx**2 + dy**2)
        if angle_step:
            radius = abs(200 / math.tan(abs(angle_step)))
            dyaw = sign_angle * step_dist / radius
        else:
            dyaw = 0

        return x + dx, y - dy, yaw + dyaw

    def parameters_of_path(self, position, target):
        x_t, y_t, yaw_t = target
        x, y, yaw = position
        dx = x_t - x
        dy = y_t - y
        dist = (dx ** 2 + dy ** 2) ** 0.5

        dxdy = abs(dx/dy)

        yaw_p = math.atan(dxdy)
        if dy >= 0:
            yaw_p += math.pi/2
        if dx <= 0:
            yaw_p *= -1

        distance_to_obstacle = 0
        if self.obstacles:
            x_obstacles = []
            y_obstacles = []
            for obstacle in self.obstacles:
                x1, y1 = obstacle[0], obstacle[1]
                x2, y2 = obstacle[2], obstacle[3]
                x3, y3 = obstacle[4], obstacle[5]
                x4, y4 = obstacle[6], obstacle[7]
                x_x_obstacle = min(map(lambda ix: abs(ix-x-50), [x1, x2, x3, x4]))
                y_y_obstacle = min(map(lambda iy: abs(iy-y-100), [y1, y2, y3, y4]))
                x_obstacles.append(x_x_obstacle)
                y_obstacles.append(y_y_obstacle)
                x_x_obstacle = min(map(lambda ix: abs(ix-x+50), [x1, x2, x3, x4]))
                y_y_obstacle = min(map(lambda iy: abs(iy-y+100), [y1, y2, y3, y4]))
                x_obstacles.append(x_x_obstacle)
                y_obstacles.append(y_y_obstacle)

            x_obstacles = min(x_obstacles)
            y_obstacles = min(y_obstacles)
            distance_to_obstacle = (x_obstacles ** 2 + y_obstacles ** 2) ** 0.5
            if distance_to_obstacle > 200:
                distance_to_obstacle = 0
            else:
                distance_to_obstacle /= 10

        tanh_path_target_distance = abs(yaw_p - yaw_t)
        tanh_path_target_distance = (2 * math.pi) % tanh_path_target_distance \
            if tanh_path_target_distance > math.pi else tanh_path_target_distance

        angle_path_target = abs(yaw_t - yaw)
        angle_path_target = (2 * math.pi) % angle_path_target \
            if angle_path_target > math.pi else angle_path_target

        return dist, tanh_path_target_distance, angle_path_target, distance_to_obstacle

    def heuristic_distance(self, position, target):
        distance, tanh_path_target_distance, angle_path_target, distance_to_obstacle = self.parameters_of_path(position.get_position(), target)
        return distance + 200 * tanh_path_target_distance + 20 * angle_path_target - distance_to_obstacle
    
    def heuristic_orientation(self, position, target):
        distance, tanh_path_target_distance, angle_path_target, distance_to_obstacle = self.parameters_of_path(position.get_position(), target)
        return 1000 * tanh_path_target_distance + 100 * angle_path_target - distance_to_obstacle
    
    def heuristic_final(self, position, target):
        distance, tanh_path_target_distance, angle_path_target, distance_to_obstacle = self.parameters_of_path(position.get_position(), target)
        return distance + 200 * tanh_path_target_distance + 100 * angle_path_target - distance_to_obstacle
    
    def check_collides(self, position, obstacles):
        x, y, _ = position.get_position()
        number_of_collisions = 0
        for obstacle in obstacles:
            polygon_position = get_polygon_from_position(position.get_position())
            polygon_obstacle = get_polygon_from_obstacle(obstacle)
            intersection = polygon_position.intersects(polygon_obstacle)
            if intersection:
                number_of_collisions += 1
                break
        if number_of_collisions > 0:
            return False
        else:
            return True

    def check_achive_target(self, position, target, heuristic, metric):
        _, _, _, distance_to_obstacle = self.parameters_of_path(position.get_position(), target)
        h_check = heuristic(position, target)
        if h_check <= metric:
            return True
        else:
            return False

    def search_by_heuristic(self, heuristic, target, metric, draw):
        count_total = 0
        current_canvas_elems = len(self.canvas.find_all())
        while True:
            target_achive = False
            count_steps = 0
            next_step_paths = []
            back = True
            best_path = None
            while not self.pq.empty():
                path = self.pq.get()
                best_path = path if not best_path else best_path
                count_steps += 1
                next_steps = self.generate_steps(path, back)
                next_step_paths.extend(next_steps)
                if count_steps >= 9:
                    break
            count_total += 1
            for step in next_step_paths:
                h_step = heuristic(step, target)
                h_len_path = step.len_path * self.step_len
                self.pq.put(step, h_step + h_len_path)

                if self.check_achive_target(step, target, heuristic, metric):
                    target_achive = True
                    find_path = step
            if count_total > 5000:
                print('SKIP ONE FAKE TARGET')
                return None

            #### DRAW STEPS ####
            if draw:
                elements_on_canvas = self.canvas.find_all()[current_canvas_elems:]
                for id_elemen in elements_on_canvas:
                    self.canvas.delete(id_elemen)
                self.root.update()
                len_path = 0
                while best_path.parent:
                    len_path += 1
                    if len_path > 0 and len_path % 20 == 0:
                        x_p, y_p, yaw_point = best_path.get_position()
                        self.canvas.create_oval(x_p+2, y_p+2, x_p-2, y_p-2, fill='yellow', outline='yellow')
                    best_path = best_path.parent
                self.root.update()

            if target_achive:
                break

        elements_on_canvas = self.canvas.find_all()[current_canvas_elems:]
        for id_elemen in elements_on_canvas:
            self.canvas.delete(id_elemen)
        self.root.update()
        return find_path

    def search_path(self, draw):
        fake_target_steps = [(10, 35, 0), (10, -35, 0),
                             (10, 35, -15), (10, 35, 15),
                             (10, -35, -15), (10, -35, 15)]
        fake_target_after_move = []
        for fake_move in fake_target_steps:
            steps, len_step, angle = fake_move
            angle = angle / 180 * math.pi
            fake_target = self.target
            for _ in range(steps):
                fake_target = self.make_move(fake_target, (len_step, angle))
            fake_target_after_move.append(fake_target)
        fake_target_after_move.append(self.target)

        pre_path = []
        print('PRESEARCH PATH')
        for i, fake_target in enumerate(fake_target_after_move):
            t = time()
            self.pq = PriorityQueue()
            self.pq.put(self.start, self.heuristic_distance(self.start, fake_target))
            self.visited_field = set()
            self.cant_move_from = set()

            path = self.search_by_heuristic(self.heuristic_distance, fake_target, 200, draw=draw)
            if path:
                h = self.heuristic_orientation(path, fake_target)
                path.len_path = 0
                self.pq = PriorityQueue()
                self.pq.put(path, h)

                path = self.search_by_heuristic(self.heuristic_orientation, fake_target, 100, draw=draw)
                print(f'TIME FOR SEARCH {i+1}\'th PREPATH: {round(time() - t, 3)} sec.')
                if path:
                    pre_path.append(path)
                    print('ONE PATH FOUND')
                points = get_polygon_from_position(fake_target, only_points=True)
                self.canvas.create_polygon(points, fill='', outline='grey52')
                self.root.update()

        print('CHOOSE PERSPECTIVE PATH')
        perspective_path = []
        for p in pre_path:
            path_and_prior = [p]
            l = 0
            h_dist = self.heuristic_final(p, self.start.get_position())
            while p.parent:
                l += 1
                p = p.parent
            path_and_prior.append(l+h_dist)
            perspective_path.append(path_and_prior)

        perspective_path.sort(key=lambda x: x[1])


        for path_found in perspective_path:
            path_found = path_found[0]
            points = get_polygon_from_position(path_found.get_position(), only_points=True)
            self.canvas.create_polygon(points, fill='', outline='white')

            print('SEARCH MAIN PATH TO TARGET')
            self.visited_field = set()
            self.cant_move_from = set()

            h = self.heuristic_final(path_found, self.target)
            path_found.len_path = 0
            self.pq = PriorityQueue()
            self.pq.put(path_found, h)

            path = self.search_by_heuristic(self.heuristic_final, self.target, 20, draw=draw)
            if path:
                return path


def rotate(points, angle, center):
    angle = math.radians(angle)
    cos_val = math.cos(angle)
    sin_val = math.sin(angle)
    cx, cy = center
    new_points = []

    for x_old, y_old in points:
        x_old -= cx
        y_old -= cy
        x_new = x_old * cos_val - y_old * sin_val
        y_new = x_old * sin_val + y_old * cos_val
        new_points.append((x_new+cx, y_new+cy))

    return new_points

def get_polygon_from_position(position, only_points=False):
    x,y,yaw = position
    points = [(x - 50, y - 100), (x + 50, y - 100), (x + 50, y + 100), (x - 50, y + 100)]
    new_points = rotate(points, yaw * 180 / math.pi, (x,y))
    if only_points:
        return new_points
    return Polygon(new_points)

def get_board_view_polygon_from_position(position):
    x,y,yaw = position
    dy = 200
    dx = dy * math.cos(math.pi/4)
    points = [(x - dx, y - dy), (x + dx, y - dy), (x + dx, y), (x - dx, y)]
    new_points = rotate(points, yaw * 180 / math.pi, (x,y))
    return Polygon(new_points)

def get_polygon_from_obstacle(obstacle):
    points = [(obstacle[0], obstacle[1]), (obstacle[2], obstacle[3]), (obstacle[4], obstacle[5]), (obstacle[6], obstacle[7])]
    return Polygon(points)

def collides(position, obstacle) :
    return get_polygon_from_position(position).intersection(get_polygon_from_obstacle(obstacle))


class Window():

    '''================= Your Main Function ================='''

    def go(self, event):
        a_star_search = AStar(StepPath(*self.get_start_position(), None, 0),
                              self.get_target_position(),
                              self.get_obstacles(),
                              self.root,
                              self.canvas)

        path = a_star_search.search_path(draw=True)
        if path:
            final_path = path.get_position()
            count_path = 0
            while path.get_parent():
                if count_path > 0 and count_path % 3 == 0:
                    points = get_polygon_from_position(path.get_position(), only_points=True)
                    x_point, y_point, yaw_point = path.get_position()
                    self.canvas.create_oval(x_point+2, y_point+2, x_point-2, y_point-2, fill='red', outline='red')
                if count_path > 0 and count_path % 15 == 0:
                    self.canvas.create_polygon(points, fill='', outline='red')
                path = path.get_parent()
                count_path += 1
            points = get_polygon_from_position(final_path, only_points=True)
            self.canvas.create_polygon(points, fill='', outline='red')

            self.root.update()
        else:
            print('PATH NOT FOUND')


    '''================= Interface Methods ================='''

    def get_obstacles(self):
        obstacles = []
        potential_obstacles = self.canvas.find_all()
        for i in potential_obstacles:
            if (i > 2) :
                coords = self.canvas.coords(i)
                if coords:
                    obstacles.append(coords)
        return obstacles


    def get_start_position(self) :
        x,y = self.get_center(2) # Purple block has id 2
        yaw = self.get_yaw(2)
        return x,y,yaw

    def get_target_position(self) :
        x,y = self.get_center(1) # Green block has id 1
        yaw = self.get_yaw(1)
        return x,y,yaw


    def get_center(self, id_block):
        coords = self.canvas.coords(id_block)
        center_x, center_y = ((coords[0] + coords[4]) / 2, (coords[1] + coords[5]) / 2)
        return [center_x, center_y]

    def get_yaw(self, id_block):
        center_x, center_y = self.get_center(id_block)
        first_x = 0.0
        first_y = -1.0
        second_x = 1.0
        second_y = 0.0
        points = self.canvas.coords(id_block)
        end_x = (points[0] + points[2])/2
        end_y = (points[1] + points[3])/2
        direction_x = end_x - center_x
        direction_y = end_y - center_y
        length = math.hypot(direction_x, direction_y)
        unit_x = direction_x / length
        unit_y = direction_y / length
        cos_yaw = unit_x * first_x + unit_y * first_y
        sign_yaw = unit_x * second_x + unit_y * second_y
        if (sign_yaw >= 0 ) :
            return math.acos(cos_yaw)
        else :
            return -math.acos(cos_yaw)

    def get_vertices(self, id_block):
        return self.canvas.coords(id_block)

    '''=================================================='''

    def rotate(self, points, angle, center):
        angle = math.radians(angle)
        cos_val = math.cos(angle)
        sin_val = math.sin(angle)
        cx, cy = center
        new_points = []

        for x_old, y_old in points:
            x_old -= cx
            y_old -= cy
            x_new = x_old * cos_val - y_old * sin_val
            y_new = x_old * sin_val + y_old * cos_val
            new_points.append(x_new+cx)
            new_points.append(y_new+cy)

        return new_points

    def start_block(self, event):
        widget = event.widget
        widget.start_x = event.x
        widget.start_y = event.y

    def in_rect(self, point, rect):
        x_start, x_end = min(rect[::2]), max(rect[::2])
        y_start, y_end = min(rect[1::2]), max(rect[1::2])

        if x_start < point[0] < x_end and y_start < point[1] < y_end:
            return True

    def motion_block(self, event):
        widget = event.widget

        for i in range(1, 10):
            if widget.coords(i) == []:
                break
            if self.in_rect([event.x, event.y], widget.coords(i)):
                coords = widget.coords(i)
                id = i
                break

        res_cords = []
        try:
            coords
        except:
            return

        for ii, i in enumerate(coords):
            if ii % 2 == 0:
                res_cords.append(i + event.x - widget.start_x)
            else:
                res_cords.append(i + event.y - widget.start_y)

        widget.start_x = event.x
        widget.start_y = event.y
        widget.coords(id, res_cords)
        widget.center = ((res_cords[0] + res_cords[4]) / 2, (res_cords[1] + res_cords[5]) / 2)

    def draw_block(self, points, color):
        x = self.canvas.create_polygon(points, fill=color)
        return x

    def distance(self, x1, y1, x2, y2):
        return ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5

    def set_id_block(self, event):
        widget = event.widget

        for i in range(1, 10):
            if widget.coords(i) == []:
                break
            if self.in_rect([event.x, event.y], widget.coords(i)):
                coords = widget.coords(i)
                id = i
                widget.id_block = i
                break

        widget.center = ((coords[0] + coords[4]) / 2, (coords[1] + coords[5]) / 2)

    def rotate_block(self, event):
        angle = 0
        widget = event.widget

        if widget.id_block == None:
            for i in range(1, 10):
                if widget.coords(i) == []:
                    break
                if self.in_rect([event.x, event.y], widget.coords(i)):
                    coords = widget.coords(i)
                    id = i
                    widget.id_block == i
                    break
        else:
            id = widget.id_block
            coords = widget.coords(id)

        wx, wy = event.x_root, event.y_root
        try:
            coords
        except:
            return

        block = coords
        center = widget.center
        x, y = block[2], block[3]

        cat1 = self.distance(x, y, block[4], block[5])
        cat2 = self.distance(wx, wy, block[4], block[5])
        hyp = self.distance(x, y, wx, wy)

        if wx - x > 0: angle = math.acos((cat1**2 + cat2**2 - hyp**2) / (2 * cat1 * cat2))
        elif wx - x < 0: angle = -math.acos((cat1**2 + cat2**2 - hyp**2) / (2 * cat1 * cat2))

        new_block = self.rotate([block[0:2], block[2:4], block[4:6], block[6:8]], angle, center)
        self.canvas.coords(id, new_block)

    def delete_block(self, event):
        widget = event.widget.children["!canvas"]

        for i in range(1, 10):
            if widget.coords(i) == []:
                break
            if self.in_rect([event.x, event.y], widget.coords(i)):
                widget.coords(i, [0,0])
                break

    def create_block(self, event):
        block = [[0, 100], [100, 100], [100, 300], [0, 300]]

        id = self.draw_block(block, "black")

        self.canvas.tag_bind(id, "<Button-1>", self.start_block)
        self.canvas.tag_bind(id, "<Button-3>", self.set_id_block)
        self.canvas.tag_bind(id, "<B1-Motion>", self.motion_block)
        self.canvas.tag_bind(id, "<B3-Motion>", self.rotate_block)

    def make_draggable(self, widget):
        widget.bind("<Button-1>", self.drag_start)
        widget.bind("<B1-Motion>", self.drag_motion)

    def drag_start(self, event):
        widget = event.widget
        widget.start_x = event.x
        widget.start_y = event.y

    def drag_motion(self, event):
        widget = event.widget
        x = widget.winfo_x() - widget.start_x + event.x + 200
        y = widget.winfo_y() - widget.start_y + event.y + 100
        widget.place(rely=0.0, relx=0.0, x=x, y=y)

    def create_button_create(self):
        button = Button(
            text="New",
            bg="#555555",
            activebackground="blue",
            borderwidth=0
        )

        button.place(rely=0.0, relx=0.0, x=200, y=100, anchor=SE, width=200, height=100)
        button.bind("<Button-1>", self.create_block)

    def create_green_block(self, center_x):
        block = [[center_x - 50, 100],
                 [center_x + 50, 100],
                 [center_x + 50, 300],
                 [center_x - 50, 300]]

        id = self.draw_block(block, "green")

        self.canvas.tag_bind(id, "<Button-1>", self.start_block)
        self.canvas.tag_bind(id, "<Button-3>", self.set_id_block)
        self.canvas.tag_bind(id, "<B1-Motion>", self.motion_block)
        self.canvas.tag_bind(id, "<B3-Motion>", self.rotate_block)

    def create_purple_block(self, center_x, center_y):
        block = [[center_x - 50, center_y - 300],
                 [center_x + 50, center_y - 300],
                 [center_x + 50, center_y - 100],
                 [center_x - 50, center_y - 100]]

        id = self.draw_block(block, "purple")

        self.canvas.tag_bind(id, "<Button-1>", self.start_block)
        self.canvas.tag_bind(id, "<Button-3>", self.set_id_block)
        self.canvas.tag_bind(id, "<B1-Motion>", self.motion_block)
        self.canvas.tag_bind(id, "<B3-Motion>", self.rotate_block)

    def create_button_go(self):
        button = Button(
            text="Go",
            bg="#555555",
            activebackground="blue",
            borderwidth=0
        )

        button.place(rely=0.0, relx=1.0, x=0, y=200, anchor=SE, width=100, height=200)
        button.bind("<Button-1>", self.go)

    def run(self):
        root = self.root

        self.create_button_create()
        self.create_button_go()
        self.create_green_block(self.width/2)
        self.create_purple_block(self.width/2, self.height)

        root.bind("<Delete>", self.delete_block)

        root.mainloop()
        
    def __init__(self):
        self.root = Tk()
        self.root.title("")
        self.width = self.root.winfo_screenwidth()
        self.height = self.root.winfo_screenheight()
        self.root.geometry(f'{self.width}x{self.height}')
        self.canvas = Canvas(self.root, bg="#777777", height=self.height, width=self.width)
        self.canvas.pack()
        # self.points = [0, 500, 500/2, 0, 500, 500]
    
if __name__ == "__main__":
    run = Window()
    run.run()
