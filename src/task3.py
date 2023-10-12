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