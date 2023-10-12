'''
def line_em_up(self):
        """!
        @brief      task3
        """
        self.current_state = "line_em_up"
        self.next_state = "idle"
        print("##################### Line 'em up Start #####################")


        self.small_counter = 0
        self.small_x = -250
        self.small_y = -25
        block_offset = 80


        print("----------------------Step1: Clear Work Space---------------------")
        while(True):
            self.sky_walker(camera_clean = True)
            blocks_need_to_be_clear = detectBlocksUsingCluster(
                                                    rgb_img= self.camera.VideoFrame.copy(),
                                                    depth_raw= self.camera.DepthFrameRaw,
                                                    boundary=self.camera.boundary[8:10],
                                                    only_blocks= False)
            if blocks_need_to_be_clear == []:
                break
            for block in blocks_need_to_be_clear:
                block_center, block_orientation = self.camera.transformFromImageToWorldFrame((block.center[1], block.center[0])), block.orientation
                if block.type == "unknown":
                    print("----------------------Remove unknown Block ---------------------")
                    block_center[2] = 25

                    self.sky_walker()
                    self.auto_pick(target_pos=block_center, depth= block.depth,
                                    block_ori = block_orientation,
                                    isbig=False, isStack=False, saveTime=False)
                    self.throw_away()


                else:
                    if block.type == "big":
                        block_center[2] = 38
                        isbig = True
                    else:
                        block_center[2] = 25
                        isbig = False


                    self.sky_walker()
                    self.auto_pick(target_pos=block_center, depth= block.depth,
                                    block_ori = block_orientation,
                                    isbig=isbig, isStack=block.stack, saveTime=False)
                    self.sky_walker()
                    self.auto_place(target_pos=[self.small_x,self.small_y,0],
                                    target_orientation = np.pi/2, isbig=isbig,
                                    save_time=False)


                    if self.small_counter % 2 == 0:
                        self.small_y = -100
                        if self.small_counter != 0:
                            self.small_x -= block_offset
                    else:
                        self.small_y = -100 + block_offset
           


        print("----------------------Step2: Start to Line Up---------------------")
        color_order = {"red": 0, "orange": 1, "yellow": 2, "green": 3, "blue": 4, "purple": 5}
        # [x,y]
        color_place_big = {"red": [-100, 275, 0], "orange": [-55, 275, 0],
                            "yellow": [-10, 275, 0], "green": [35, 275, 0],
                            "blue": [80, 275, 0], "purple": [125, 275, 0]}


        color_place_small = {"red": [-150,200, 0], "orange": [-110,200, 0],
                            "yellow": [-70,200, 0], "green": [-30,200, 0],
                            "blue": [10,200, 0], "purple": [50,200, 0]}
        color_line_big = {"red": False, "orange": False,
                        "yellow": False, "green": False,
                        "blue": False, "purple": False}
        color_line_small = {"red": False, "orange": False,
                        "yellow": False, "green": False,
                        "blue": False, "purple": False}
        while(True):
            if all(color_line_big.values()) and all(color_line_small.values()):
                print("--------------------Mission Complete---------------------")
                break
           
            self.sky_walker(camera_clean = True)
            self.camera.blocks = detectBlocksUsingCluster(self.camera.VideoFrame.copy(), 
                                                          self.camera.DepthFrameRaw,
                                                          boundary=self.camera.boundary[10:13],
                                                          only_blocks=False)
            self.camera.blocks = sorted(self.camera.blocks, key=lambda x: color_order.get(x.color, len(color_order)))
            sorted_blocks = []
            for block in self.camera.blocks:
                if block.type == "unknown":
                    continue
                else:
                    sorted_blocks.append(block)


            if sorted_blocks == []:
                print("--------------------There is no more blocks---------------------")
                print("--------------------Detect Again---------------------")
                continue


            for block in sorted_blocks:
                block_center, block_orientation = self.camera.transformFromImageToWorldFrame((block.center[1], block.center[0])), block.orientation
               
                if block.type == "big":
                    block_center[2] = 38
                    isbig = True
                else:
                    block_center[2] = 25
                    isbig = False
                if block.stack == True:
                    block_center[2] = block.depth
                self.sky_walker()
                self.auto_pick(target_pos=block_center, depth= block.depth,
                                block_ori = block_orientation,
                                isbig=isbig, isStack=block.stack, saveTime=False)


                if self.rxarm.get_gripper_position() > 0.02:
                    # Big block
                    if not color_line_big[block.color]:
                        target_pos = color_place_big[block.color]
                        self.sky_walker()
                        self.auto_place(target_pos= target_pos, target_orientation = np.pi/2,isbig = True, save_time=False)
                        color_line_big[block.color] = True
                    else:
                        print("----------------------We fuck up, something wrong---------------------")
                        self.throw_away()
                else:
                    # Small block
                    if not color_line_small[block.color]:
                        target_pos = color_place_small[block.color]
                        self.sky_walker()
                        self.auto_place(target_pos= target_pos, target_orientation = np.pi/2,isbig = False, save_time=False)
                        color_line_small[block.color] = True
                    else:
                        print("----------------------We fuck up, something  wrong---------------------")
                        self.throw_away()


        self.initialize_rxarm()
        time.sleep(0.5)
        self.rxarm.arm.go_to_sleep_pose(moving_time = 1.5,
                            accel_time=0.5,
                            blocking=True)
        print("##################### Line 'em up Complete #####################")

'''

    # # Event 3:Line 'em up!
    # def line_em_up(self):
    #     """!
    #     @brief      task3
    #     """
    #     self.current_state = "line_em_up"
    #     self.next_state = "idle"
    #     print("##################### Line 'em up Start #####################")
    #     is_start = False
    #     while(not is_start):
    #         self.pick_n_sort(istask3=True)
    #         print("----------------------Clear Stage Complete---------------------")
            
    #         print("----------------------Double check negative---------------------")
    #         self.camera.blocks = detectBlocksUsingCluster(self.camera.VideoFrame.copy(), self.camera.DepthFrameRaw,boundary=self.camera.boundary[6:8]) 



    #         color_detect_big = {"red": False, "orange": False, "yellow": False, "green": False, "blue": False, "purple": False}
    #         color_detect_small = {"red": False, "orange": False, "yellow": False, "green": False, "blue": False, "purple": False}
            
    #         sorted_blocks = []
    #         for block in self.camera.blocks:
    #             self.block_center, self.block_orientation = self.camera.transformFromImageToWorldFrame((block.center[1], block.center[0])),block.orientation 

    #             if self.block_center[0] > 0:
    #                 color_detect_big[block.color] = True
    #                 block.type = "big"
    #                 sorted_blocks.append(block)
                    
    #             else:
    #                 color_detect_small[block.color] = True
    #                 block.type = "small"
    #                 sorted_blocks.append(block)

    #         is_start = True
    #         for i in color_detect_big.values():
    #             if i == False:
    #                 is_start = False
    #         for i in color_detect_small.values():
    #             if i == False:
    #                 is_start = False
    #     print("----------------------Double check complete---------------------")

    #     print("----------------------Start line up ---------------------")

    #     self.initialize_rxarm()
    #     time.sleep(0.5)

    #     while self.camera.blocks == None:
    #         print("There is no blocks in the workspace!!")
    #         time.sleep(1)
        

    #     # Sort the list of blocks by color

    #     # Define a custom order for colors
    #     # color_order = {"red": 0, "orange": 1, "yellow": 2, "green": 3, "blue": 4, "purple": 5, None:6}
    #     # sorted_blocks = sorted(blocks, key=lambda x: color_order.get(x.color, len(color_order)))
    #     self.small_x, self.big_x = -150, -150
    #     self.small_y, self.big_y = 200, 275
    #     block_counter,small_block_counter,big_block_counter = 1,1,1
        
    #     for block in sorted_blocks:
    #         self.block_center, self.block_orientation = self.camera.transformFromImageToWorldFrame((block.center[1], block.center[0])),block.orientation 

    #         # Filter out possible mis-ditection
    #         if self.block_center[2] < 65: 
    #             # print(block_center)
    #             # Assume all non-sorted blocks are in the positive plane:    
    #             print("--------------------- start a block:No.",block_counter,"---------------------------")
    #             # Line small blocks in color order in the left negative plane
    #             if block.type == "small":
    #                 print("================ Small Block No.",small_block_counter," ",
    #                     block.color,"=========================")
                    
    #                 if block.color == 'red':
    #                     self.small_x = -150
    #                 elif block.color == 'orange':
    #                     self.small_x = -100
    #                 elif block.color == 'yellow':
    #                     self.small_x = -50
    #                 elif block.color == 'green':
    #                     self.small_x = 0
    #                 elif block.color == 'blue':
    #                     self.small_x = 50
    #                 elif block.color == 'purple':
    #                     self.small_x = 100
                        
    #                 self.auto_pick(target_pos=self.block_center,block_ori = self.block_orientation)
                    
    #                 # to the sky
    #                 target_joint = [-np.pi/4,-np.pi/5,-np.pi/4,0,0]
    #                 current_joint_angles = self.rxarm.get_positions()
    #                 displacement = np.array(target_joint) - current_joint_angles

    #                 if np.max(abs(displacement)) > np.pi/2:
    #                     # self.initialize_rxarm()
    #                     # time.sleep(0.5)    
                        
    #                     displacement_unit = displacement/3
    #                     pre_pos1 = current_joint_angles + displacement_unit
    #                     pre_pos2 = current_joint_angles + displacement_unit * 2

    #                     move_time,ac_time = self.calMoveTime(pre_pos1)
    #                     self.rxarm.arm.set_joint_positions(pre_pos1,
    #                                             moving_time = move_time+0.5, 
    #                                             accel_time = ac_time,
    #                                             blocking = True)
    #                     move_time,ac_time = self.calMoveTime(pre_pos2)
    #                     self.rxarm.arm.set_joint_positions(pre_pos2,
    #                                             moving_time = move_time+0.5, 
    #                                             accel_time = ac_time,
    #                                             blocking = True)
    #                 elif np.max(abs(displacement)) > np.pi/3:

    #                     displacement_unit = displacement/2
    #                     pre_pos1 = current_joint_angles + displacement_unit
    #                     move_time,ac_time = self.calMoveTime(pre_pos1)
    #                     self.rxarm.arm.set_joint_positions(pre_pos1,
    #                                             moving_time = move_time+0.5, 
    #                                             accel_time = ac_time,
    #                                             blocking = True)

    #                 move_time,ac_time = self.calMoveTime(target_joint)
    #                 self.rxarm.arm.set_joint_positions(target_joint,
    #                                             moving_time = move_time*1.5, 
    #                                             accel_time = ac_time,
    #                                             blocking = True)
    #                 print("Auto Place: Reach Pos1")
    #                 time.sleep(1.2)

    #                 self.auto_place(target_pos=[self.small_x,self.small_y,5],target_orientation = 0)
    #                 self.small_x += 50
    #                 small_block_counter += 1
    #             # Line big blocks in color order in the right negative plane
    #             elif block.type == "big":
    #                 print("================ Big Block No.",big_block_counter," ",
    #                     block.color,"=========================")
                    
    #                 if block.color == 'red':
    #                     self.big_x = -100
    #                 elif block.color == 'orange':
    #                     self.big_x = -50
    #                 elif block.color == 'yellow':
    #                     self.big_x = 0
    #                 elif block.color == 'green':
    #                     self.big_x = 50
    #                 elif block.color == 'blue':
    #                     self.big_x = 100
    #                 elif block.color == 'purple':
    #                     self.big_x = 150

    #                 self.auto_pick(target_pos=self.block_center,block_ori = self.block_orientation)

    #                 target_joint = [-np.pi/4,-np.pi/5,-np.pi/4,0,0]
    #                 current_joint_angles = self.rxarm.get_positions()
    #                 displacement = np.array(target_joint) - current_joint_angles

    #                 if np.max(abs(displacement)) > np.pi/2:
    #                     # self.initialize_rxarm()
    #                     # time.sleep(0.5)    
                        
    #                     displacement_unit = displacement/3
    #                     pre_pos1 = current_joint_angles + displacement_unit
    #                     pre_pos2 = current_joint_angles + displacement_unit * 2

    #                     move_time,ac_time = self.calMoveTime(pre_pos1)
    #                     self.rxarm.arm.set_joint_positions(pre_pos1,
    #                                             moving_time = move_time+0.5, 
    #                                             accel_time = ac_time,
    #                                             blocking = True)
    #                     move_time,ac_time = self.calMoveTime(pre_pos2)
    #                     self.rxarm.arm.set_joint_positions(pre_pos2,
    #                                             moving_time = move_time+0.5, 
    #                                             accel_time = ac_time,
    #                                             blocking = True)
    #                 elif np.max(abs(displacement)) > np.pi/3:

    #                     displacement_unit = displacement/2
    #                     pre_pos1 = current_joint_angles + displacement_unit
    #                     move_time,ac_time = self.calMoveTime(pre_pos1)
    #                     self.rxarm.arm.set_joint_positions(pre_pos1,
    #                                             moving_time = move_time+0.5, 
    #                                             accel_time = ac_time,
    #                                             blocking = True)

    #                 move_time,ac_time = self.calMoveTime(target_joint)
    #                 self.rxarm.arm.set_joint_positions(target_joint,
    #                                             moving_time = move_time*1.5, 
    #                                             accel_time = ac_time,
    #                                             blocking = True)
    #                 print("Auto Place: Reach Pos1")
    #                 time.sleep(1.2)

    #                 self.auto_place(target_pos=[self.big_x,self.big_y,5],target_orientation = 0)
    #                 self.big_x += 50
    #                 big_block_counter += 1
    #         print("--------------------- start a block:No.",block_counter,"---------------------------")
    #         time.sleep(0.2)

    #     self.initialize_rxarm()
    #     time.sleep(0.5)
    #     self.rxarm.arm.go_to_sleep_pose(moving_time = 1.5,
    #                         accel_time=0.5,
    #                         blocking=True)
    #     print("##################### Line 'em up Complete #####################")
    #     pass
    