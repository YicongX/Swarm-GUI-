import sys
import time
import numpy as np

from PyQt5.QtWidgets import QApplication, QMessageBox
from PyQt5.QtCore import QThread, pyqtSignal
from viewer import Viewer
from model import Model


class SwarmThread(QThread, Model):
    signal = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.is_running = False
        self.ok_count = 0
        self.dis_error = 0

# Swarm status can be determined by comparing distance of current location from destination to a set small value in this case 0.01
# If smaller then mission accomplished, emit 'OK' signal, is not emit 'running' signal
    def run(self):
        while True:
            if self.is_running:
                self.run_mission(motion_noise=0.1)
                self.dis_error = self.calculate_dis_error()
                if self.dis_error < self.dis_error_thr:
                    self.ok_count += 1
                    self.signal.emit('OK' + str(self.ok_count))
                else:
                    self.signal.emit('running')
                time.sleep(0.1)
            else:
                break


class Controller:
    def __init__(self):
# Connect GUI elemnts to control functions
        self.viewer = Viewer()
        self.model = SwarmThread()
        self.desired_formation = 'circle'

        self.has_started = False
        
        self.model.init_swarm()

        self.viewer.pushButton_start.clicked.connect(self.start_mission)
        self.viewer.pushButton_stop.clicked.connect(self.stop_mission)

        # upadte desired formation
        self.pre_destination_x = self.viewer.doubleSpinBox_des_x.value()
        self.pre_destination_y = self.viewer.doubleSpinBox_des_y.value()
        self.destination_x = self.viewer.doubleSpinBox_des_x.value()
        self.destination_y = self.viewer.doubleSpinBox_des_y.value()
        self.error_vec_x = self.destination_x - self.pre_destination_x
        self.error_vec_y = self.destination_y - self.pre_destination_y
        
        self.viewer.pushButton_f_circle.clicked.connect(
            self.update_desired_formation)
        self.viewer.pushButton_f_square.clicked.connect(
            self.update_desired_formation)
        self.viewer.pushButton_f_triangle.clicked.connect(
            self.update_desired_formation)
        self.viewer.pushButton_f_random.clicked.connect(
            self.update_desired_formation)
        # destination
        self.viewer.doubleSpinBox_des_x.valueChanged.connect(
            self.update_destination)
        self.viewer.doubleSpinBox_des_y.valueChanged.connect(
            self.update_destination)
        self.viewer.clicked_pos.connect(self.update_destination_by_click)
        # update robot number
        self.viewer.pushButton_set_n_r.clicked.connect(self.update_robot_number)
        # update boundary
        self.viewer.pushButton_set_boundary.clicked.connect(self.update_map_boundaries)

        self.model.signal.connect(self.swarm_mission_callback)
        self.viewer.update_system_info('Please set number of robots and map boundary.', 'warning')

# Start mission when user click start button. Run model.py, model.py is a thread, is running all the time but only emit output when signal is 'running'
    def start_mission(self):
        self.model.ok_count = 0
        self.has_started = True
        # read parameter from the GUI
        # self.model.n_r = self.viewer.spinBox_num_robots.value()

        # initialize swarm
        # self.model.init_initial_positions()
        # self.model.init_swarm(self.desired_formation)
        # process bar, distance error -> pbar_value: pbar_value = k*dis_err + b
        dis = self.model.calculate_dis_error()
        self.pbar_value_k = 50 / (self.model.dis_error_thr - dis)
        self.pbar_value_b = 50 - self.model.dis_error_thr * self.pbar_value_k

        self.viewer.update_desired_pos(self.model.C_d)
        # self.viewer.update_map(self.model.map_hat.map)
        
        self.destination_x = self.viewer.doubleSpinBox_des_x.value()
        self.destination_y = self.viewer.doubleSpinBox_des_y.value()
        
        self.error_vec_x = self.destination_x - self.pre_destination_x
        self.error_vec_y = self.destination_y - self.pre_destination_y

        self.model.is_running = True
        self.model.start()
        self.viewer.update_system_info('Mission started!', 'info')

# Update swarm formation when user click differnt formation button
    def update_desired_formation(self):
        # move the selected sign
        style_sheet = '''QPushButton{border-color:Transparent;
                    background-color: rgb(223, 223, 223);
                    color:white;
                    border-width: 1px;       
                    border-style: solid;
                    border-radius: 5px;
                    margin:2px;}'''
        self.viewer.pushButton_f_circle.setStyleSheet(style_sheet)
        self.viewer.pushButton_f_square.setStyleSheet(style_sheet)
        self.viewer.pushButton_f_random.setStyleSheet(style_sheet)
        self.viewer.pushButton_f_triangle.setStyleSheet(style_sheet)

        sender = self.viewer.sender()
        sender.setStyleSheet('''QPushButton{border-color:black;
                            background-color: rgb(223, 223, 223);
                            color:white;
                            border-width: 2px;       
                            border-style: solid;
                            border-radius: 5px;
                            margin:2px;}''')
        self.desired_formation = sender.objectName().split('_')[-1]
        
        if self.has_started:
            # ave_pos = self.model.calculate_ave_position()
            ave_pos = np.array([self.pre_destination_x, 
                                self.pre_destination_y])
            self.model.init_formations(self.desired_formation, 
                                       center=ave_pos)
        else:
            self.model.init_formations(self.desired_formation)

        self.viewer.update_desired_pos(self.model.C_d)
        self.viewer.update_system_info('Formation set, please set destionation.')
        # self.viewer.pushButton_f_circle.set

# Update mission destination, obtian destination from view.py, viewer.py will tell where is the clicked location
    def update_destination_by_click(self, pos):
        if self.model.is_running:
            pass
        else:
            s = 'Set [{:4.2f}, {:4.2f}] as the destination?'.format(pos[0], pos[1])   
            r = QMessageBox.information(self.viewer, 'Info',
                                        s, QMessageBox.Yes | QMessageBox.No)
            if r == QMessageBox.Yes:
                self.viewer.doubleSpinBox_des_x.setValue(pos[0])
                self.viewer.doubleSpinBox_des_y.setValue(pos[1])
                self.viewer.update_system_info(s.replace('?','.') + 'please start the mission.', 'info')

# Update destnation when the destination spin box values are changed, it is used for supplymentary destination setting
    def update_destination(self, pos):
        self.viewer.update_select_point([self.viewer.doubleSpinBox_des_x.value(),
                                         self.viewer.doubleSpinBox_des_y.value()])

# Set number of robots in the swarm, function disabled when user confirm their first input
    def update_robot_number(self):
        n_r = self.viewer.spinBox_num_robots.value()
        r = QMessageBox.warning(
            self.viewer, 'warning',
            'Are you sure to set robot number as {}? It cannot be changed.'
            .format(n_r), QMessageBox.Yes | QMessageBox.No)
        if r == QMessageBox.Yes:
            self.model.n_r = n_r
            self.model.init_swarm(self.desired_formation)
            self.viewer.update_desired_pos(self.model.C_d)
            self.viewer.spinBox_num_robots.setDisabled(True)
            self.viewer.pushButton_set_n_r.setDisabled(True)
            self.viewer.update_current_robot_pos(self.model.C_t)
            l_dis = self.model.calculate_longest_robot_dis()
            self.viewer.lcdNumber_l_dis.display(l_dis)
            self.viewer.update_system_info('Robot number set to {}, please set formation'.format(n_r))

# Set map boundary, limits of x and y axis
    def update_map_boundaries(self):
        x_min = self.viewer.doubleSpinBox_x_min.value()
        y_min = self.viewer.doubleSpinBox_y_min.value()
        x_max = self.viewer.doubleSpinBox_x_max.value()
        y_max = self.viewer.doubleSpinBox_y_max.value()
        
        r = QMessageBox.warning(
            self.viewer, 'warning',
            'Are you sure to set map boundaries as {}? It cannot be changed.'
            .format([x_min, x_max, y_min, y_max]), QMessageBox.Yes | QMessageBox.No)
        if r == QMessageBox.Yes:
            self.model.boundary = [x_min, x_max, y_min, y_max]
            self.model.init_map()
            #self.model.init_initial_positions()
            self.viewer.map_fig.setXRange(x_min, x_max)
            self.viewer.map_fig.setYRange(y_min, y_max)
            # disable the button
            self.viewer.doubleSpinBox_x_min.setDisabled(True)
            self.viewer.doubleSpinBox_x_max.setDisabled(True)
            self.viewer.doubleSpinBox_y_min.setDisabled(True)
            self.viewer.doubleSpinBox_y_max.setDisabled(True)
            self.viewer.pushButton_set_boundary.setDisabled(True)
            self.viewer.update_system_info('Boundaries set to {},please set formation'.format([x_min, x_max, y_min, y_max]))
        
    def stop_mission(self):
        self.model.is_running = False
        # recode the previous destination to calculate new error vector 
        # to get new desired positions
        self.pre_destination_x = self.viewer.doubleSpinBox_des_x.value()
        self.pre_destination_y = self.viewer.doubleSpinBox_des_y.value()
        self.viewer.update_system_info('Mission Completed! Please set the Formation and Destination for the next mission.')

# Excute while mission in progress, calculate progress bar related information and display robot status including speed, futhest distance while in mission
    def swarm_mission_callback(self, sig):
        if sig.startswith('OK'):
            if sig[-1] == '1':
                # formation finished
                # change destination
                self.model.C_d[:, 0:2] += np.array(
                    [self.error_vec_x, self.error_vec_y])
                # process bar, distance error -> pbar_value: pbar_value = k*dis_err + b
                dis = self.model.calculate_dis_error()
                self.pbar_value_k = 100 / (self.model.dis_error_thr - dis)
                self.pbar_value_b = 150 - self.model.dis_error_thr * self.pbar_value_k
                self.viewer.update_desired_pos(self.model.C_d)
            elif sig[-1] == '2':
                # finished
                self.stop_mission()
            # self.viewer.update_robots_trajectories(self.model.robot_p_history)
        elif sig == 'running':
            l_dis = self.model.calculate_longest_robot_dis()
            self.viewer.lcdNumber_l_dis.display(l_dis)
            speed = self.model.calculate_speed()
            self.viewer.lcdNumber_speed.display(speed)
            self.viewer.update_current_robot_pos(self.model.C_t_hat)
            self.update_process_bar(self.model.dis_error)
            if l_dis > 0.29:
                self.viewer.textEdit_status.insertPlainText('The furthest robot is offline.\n')
        else:
            pass

# Update progress bar display 
    def update_process_bar(self, des_err):
        p_value = self.pbar_value_k * des_err + self.pbar_value_b
        self.viewer.progressBar_mission.setValue(int(p_value))
        if p_value == '1':
        	self.viewer.update_system_info('Mission Completed! Please set the Formation and Destination for the next mission.')


if __name__ == '__main__':
    app = QApplication(sys.argv)
    ctl = Controller()
    ctl.viewer.show()
    sys.exit(app.exec_())