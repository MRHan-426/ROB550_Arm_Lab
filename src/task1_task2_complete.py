'''
    # Event 1:Pick'n sort!
    def pick_n_sort(self,istask3 = False):
        if not istask3:
            self.current_state = "pick_n_sort"
            self.next_state = "idle"
            print("##################### Pick 'n sort Start #####################")
               
        self.block_number_counter = 1
        self.small_counter, self.big_counter = 0,0
        self.small_x , self.big_x = -160,160
        self.small_y,self.big_y = -120,-120

        if istask3:
            block_offset = 80
            safe_area_y = 0
            boundary = self.camera.boundary[4:6]

        else:
            boundary = self.camera.boundary[2:4]
            block_offset = 70
            safe_area_y = 0

        # 1st time deal with 2 floor blocks, 2nd time deal with general blocks
        while True:
            # Detect blocks in the plane
            self.camera.blocks = detectBlocksUsingCluster(self.camera.VideoFrame.copy(), self.camera.DepthFrameRaw,boundary=boundary)
            time.sleep(1)
            while self.camera.blocks == None:
                print("There is no blocks in the workspace!!")
                time.sleep(1)
            
            # all_block_in_neg_plane = True
            valid_blocks = []
            for block in self.camera.blocks:
                if block.type == "small" or block.type == "big":
                    valid_blocks.append(block)
                    # block_center, block_orientation = self.camera.transformFromImageToWorldFrame((block.center[1], block.center[0])),block.orientation 
                    # if block_center[1] > 0:
                    #     all_block_in_neg_plane = False
            if valid_blocks == []:
                break

            # Initialize place positions - x coordinates           
            self.initialize_rxarm()
            time.sleep(2)

            # for block in self.camera.blocks:
            for block in valid_blocks:

                block_center, block_orientation = self.camera.transformFromImageToWorldFrame((block.center[1], block.center[0])),block.orientation 

                if block_center[2] < 150: 
                    print("--------------------- start a block:No.",self.block_number_counter,"---------------------------")
                    print("block_depth", block.depth)

                    # Move small blocks in right plane to left planet
                    # if block_center[2] > 65:
                        # if it's stack 2 high, place the top one first
                        # print("=========== Stack Two ",block_center," =============")
                        # self.auto_pick(target_pos=block_center,block_ori = block_orientation,isbig=(block.type == "big"))
                                                
                        # time.sleep(0.2)
                        # current_position = self.compute_ee_world_pos()
                        # self.auto_change_position(current_pos=current_position)
                        # print("=========== Stack Two Comptele =============")
                    
                    # else:
                    # if it's stack 1 high, pick and place normally
                    if block.type == "small":
                        if block_center[0] >= 0 or block_center[1] >= safe_area_y:
                            print("=========== Small ",block_center," =============")
                            if block.stack == False:
                                block_center[2] = 25
                            else:
                                block_center[2] = block.depth
                            self.auto_pick(target_pos=block_center,block_ori = block_orientation,isbig=False, isStack = block.stack, depth=block.depth)
                            time.sleep(0.2)
                            print("gripper position:", self.rxarm.get_gripper_position())

                            if self.rxarm.get_gripper_position() > 0.02:
                                print("Actually it is a big block")
                                if self.big_counter % 2 == 0:
                                    self.big_y = -100
                                    if self.big_counter !=0:
                                        self.big_x += block_offset
                                else:
                                    self.big_y = -100 + block_offset

                                self.auto_place(target_pos=[self.big_x,self.big_y,0],target_orientation = 0,isbig = True,save_time=True)
                                self.big_counter +=1
                                self.block_number_counter += 1
                            # compute the place position   
                            if self.small_counter % 2 == 0:
                                self.small_y = -100
                                if self.small_counter != 0:
                                    self.small_x -= block_offset
                            else:
                                self.small_y = -100 + block_offset

                            self.auto_place(target_pos=[self.small_x,self.small_y,0],target_orientation = 0,isbig=False,save_time=True)
                            self.small_counter += 1
                            self.block_number_counter += 1
                    # Move big blocks in left plane to right plane
                    elif block.type == "big":
                        if block_center[0] <= 0 or block_center[1] >= safe_area_y:
                            print("=========== Big ",block_center," =============")
                            if block.stack == False:
                                block_center[2] = 38
                            else:
                                block_center[2] = block.depth
                            self.auto_pick(target_pos=block_center,block_ori = block_orientation,isbig=True ,isStack = block.stack, depth = block.depth)
                            time.sleep(0.2)
                            print("gripper position:", self.rxarm.get_gripper_position())

                            if self.rxarm.get_gripper_position() < 0.02:
                                print("Actually it is a small block")
                                # compute the place position   
                                if self.small_counter % 2 == 0:
                                    self.small_y = -100
                                    if self.small_counter != 0:
                                        self.small_x -= block_offset
                                else:
                                    self.small_y = -100 + block_offset

                                self.auto_place(target_pos=[self.small_x,self.small_y,0],target_orientation = 0,isbig=False,save_time=True)
                                self.small_counter += 1
                                self.block_number_counter += 1
                            else:
                                # compute the place position
                                if self.big_counter % 2 == 0:
                                    self.big_y = -100
                                    if self.big_counter !=0:
                                        self.big_x += block_offset
                                else:
                                    self.big_y = -100 + block_offset

                                self.auto_place(target_pos=[self.big_x,self.big_y,0],target_orientation = 0,isbig = True,save_time=True)
                                self.big_counter +=1
                                self.block_number_counter += 1
                    time.sleep(0.2)
                    # self.initialize_rxarm()
                    # time.sleep(0.5)
                    # self.rxarm.arm.go_to_sleep_pose(moving_time = 1.5,
                    #         accel_time=0.5,
                    #         blocking=True)
                    print("--------------------- end a block ---------------------------")
                    # self.safe_pos()
                else:
                    print("This is April Tag!")
            # self.initialize_rxarm()
            # time.sleep(0.5)
            # self.rxarm.arm.go_to_sleep_pose(moving_time = 1.5,
            #                   accel_time=0.5,
            #                   blocking=True)
        print("##################### Pick 'n sort finished #####################")   
'''