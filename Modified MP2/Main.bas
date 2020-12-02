
Dim PI As Double					   'PI constant for calculations
Dim mobot As Integer				   'Mobot name
Dim node As Integer					   'node in the graph
Dim last_node As Integer			   'last node on current graph
Dim adj_mat(30,30) As Integer          'adjacency matrix, describes graph of maze
Dim node_array(30,4) As Integer        'position of each node relative to start pos
									   'constructed as: (x,y,visited?,junction?) (-1,0,1,1) note: 1 is visited/junction cell

Dim steps As Integer				   'step counter for backtracking
Dim step_q(10) As Integer			   'step queue for replaying steps
									   'each steps are represented as follows:
									   'L = 1
									   'R = 2
									   'F = 3
									   'B = 4

Dim curr_pos(2) As Integer			   'Current robot position (x,y)



Sub Main
	SetMobotPosition(0,4.5,7.5,0)

	SetTimeStep(1)



	Dim bck_trck As Integer				   'back tracking flag for DFS
	Dim start_cell As Integer			   'flag just incase we start facing a dead end

	PI = 4 * Atn(1)						   'initialize constant and variables
	start_cell = 1
	mobot = 0
	node = 0
	bck_trck = 0
	curr_pos(0) = 0
	curr_pos(1) = 0
	steps_q = Array(0,0,0,0,0,0,0,0,0,0)


	For cell = 0 To 40

		print_currpos()

		If(IsVisited(node) = 0 And bck_trck = 0) Then

			'--------------------------------Unvisited exploring---------------------------------

			walls = check_cell(mobot)	'walls = { R,F,L,B } POV of mobot


			If(walls(1) = 0 )Then		'if no wall infront then dont turn
			ElseIf(walls(2) = 0 ) Then
				rotate90(mobot,"left")	'if wall no wall on the left,face left
			ElseIf(walls(0) = 0) Then
				rotate90(mobot,"right")	'if wall no wall on the right,face right
			Else
				rotate90(mobot,"left")	'turn around if deadend
				rotate90(mobot,"left")

				If(start_cell = 0)Then	'if we start out facing the wall then don't assert the
					bck_trck = 1		'back track flag, else assert back track flag
				Else
					bck_trck = 0
					start_cell = 0
				End If					'and assume that we back track at each dead end
			End If



			'record node position in grid
			node_array(node,0) = curr_pos(0)	'record x
			node_array(node,1) = curr_pos(1)	'record y
			node_array(node,2) = 1				'label as visited
			If(wall_cnt(mobot) < 2) Then
				node_array(node,3) = 1			'label if junction
			End If


			'if not back tracking do the following
			If(bck_trck = 0)Then

											'update adjacency matrix
				adj_mat(node,node + 1) = 1  'add edge from this node to the next if not back tracking
				adj_mat(node + 1, node) = 1 'add edge on the other node index as well

				mv_frwd_1cell(mobot)	   'move mobot forward to next node

				update_currpos(mobot)	'update position vector after moving

				steps = steps + 1		   'update step count
				node = node + 1			   'update node number


														'record previous steps
														'note: walls = { R,F,L,B }
				If(walls(1) = 0)Then					'if no wall infront then we didnt turn
					step_q(steps) = 3
				ElseIf(walls(2) = 0 ) Then
					step_q(steps) = 1					'if wall no the left then we turned left
				ElseIf(walls(0) = 0) Then
					step_q(steps) = 2   				'if wall on the right then we turned right
				End If


			Else

				mv_frwd_1cell(mobot)	    'move mobot back to last node to start backtrack routine
				update_currpos(mobot)		'update position vector after moving

				last_node = node			'record last node number
				node = node - 1				'update node number

				'print_stepq(steps)			'print out last steps in the queue
			End If

			'--------------------------------------------------------------------------------
		ElseIf(bck_trck = 1 And steps > 0)Then


			'print_currpos()
			'check walls if two paths are available/only one adjacent wall
			If(wall_cnt(mobot) < 2)Then

				walls = check_cell(mobot)

				If(walls(1) = 0 And IsNeighborVisited(mobot,"forward") = 0)Then		'dont turn
				ElseIf(walls(0) = 0 And IsNeighborVisited(mobot,"right") = 0)Then
					rotate90(mobot,"right")											'face right
				ElseIf(walls(2) = 0 And IsNeighborVisited(mobot,"left") = 0)Then
					rotate90(mobot,"left")											'face left
				End If

				mv_frwd_1cell(mobot)
				update_currpos(mobot)


											     'update adjacency matrix
				adj_mat(node,last_node + 1) = 1  'add edge from this node to the new node on the new branch
				adj_mat(last_node + 1, node) = 1 'add edge on the counter part

				node = last_node + 1
												 'reset flags and step count
				bck_trck = 0
				steps = 0

			Else

				'search current position on node list and check if node exists on available path


				'------------------backtrack nodes---------------
				Select Case step_q(steps)
					Case 1 'previous was left so turn right
						rotate90(mobot,"right")	'face right
					Case 2 'previous was right so turn left
						rotate90(mobot,"left")	'face left
				End Select

				mv_frwd_1cell(mobot)	   'move mobot forward to next node
				update_currpos(mobot)	   'update position vector after moving

				steps = steps - 1		   'update step number
				node = node - 1			   'update node number since we moved back a node

				'-------------------------------------------------


			End If
		End If
	Next


End Sub

'convert degrees to radians
Function ToRadians(degrees As Integer) As Double
	ToRadians = degrees * (PI / 180)
End Function

'print node array
Function print_nodearray()
	Dim node_str As String

	node_str = "Current Node: " +CStr(node)+" (" + CStr(curr_pos(0)) + "," + CStr(curr_pos(1)) + ")" +vbNewLine

	For i = 0 To 30
		node_str = node_str + "Node: " + CStr(i) + " (" + CStr(node_array(i,0)) + "," + CStr(node_array(i,1)) + ")" +vbNewLine
	Next

	MsgBox(node_str)
End Function


'print current position vector
Function print_currpos()

	MsgBox("Node:"+ CStr(node)+" (" + CStr(curr_pos(0)) + "," + CStr(curr_pos(1)) + ")")
End Function

'print step queue as FIFO
Function print_stepq(steps As Integer)
	Dim step_str As String

	step_str = "Step_q : "

	For i = 0 To steps
		Select Case step_q(steps - i)
			Case 1
				step_str = step_str + "L "
			Case 2
				step_str = step_str + "R "
			Case 3
				step_str = step_str + "F "
		End Select

	Next

	MsgBox(step_str)

End Function

'update position vector after moving 1 cell
Function update_currpos(mobot As Integer) As Integer

		'update position based on where the robot is facing
		theta = ToRadians(GetMobotTheta(mobot))

		curr_pos(0) = curr_pos(0) + CInt(Cos(theta))
		curr_pos(1) = curr_pos(1) + CInt(Sin(theta))

End Function

'check if node is labelled as a junction
Function IsNodeJunction(node As Integer) As Integer

	Dim flag As Integer
	flag = 0
	If(node_array(node,4) = 1)Then
		flag = 1
	End If

	IsNodeJunction = flag
End Function


'check neighboring cell on the prescribed direction (POV of mobot) if it's been visited
Function IsNeighborVisited(mobot As Integer,dir_str As String) As Integer

	Dim flag As Integer
	Dim x As Integer
	Dim y As Integer
	flag = 0

	'get position values
	theta = ToRadians(GetMobotTheta(mobot))
	x = curr_pos(0)
	y = curr_pos(1)

	If(dir_str = "left")Then
		For i = 0 To 30
			If(InNodeArray(x - Sin(theta), y + Cos(theta)))Then
				flag = 1
			End If
		Next
	ElseIf(dir_str = "right")Then
		For i = 0 To 30
			If(InNodeArray(x + Sin(theta), y - Cos(theta)))Then
				flag = 1
			End If
		Next
	ElseIf(dir_str = "forward")Then
		For i = 0 To 30
			If(InNodeArray(x + Cos(theta), y + Sin(theta)))Then
				flag = 1
			End If
		Next
	ElseIf(dir_str = "back")Then
		For i = 0 To 30
			If(InNodeArray(x - Cos(theta), y - Sin(theta)))Then
				flag = 1
			End If
		Next
	End If

	IsNeighborVisited = flag

End Function

'check if coordinates match any node
Function InNodeArray(x As Variant, y As Variant) As Integer
	Dim flag As Integer
	flag = 0

	'linear search
	For i = 0 To 30
		If(node_array(i,0) = CInt(x) And node_array(i,1) = CInt(y))Then
			flag = 1
		End If
	Next

	InNodeArray = flag
End Function

'count walls adjacent to the mobot
Function wall_cnt(mobot As Integer) As Integer
	walls = check_cell(mobot)
	cnt = 0
	For i = 0 To 3
		If(walls(i))Then
			cnt = cnt + 1
		End If
	Next

	wall_cnt = cnt
End Function




Function IsVisited(node As Integer) As Integer	'for checking if the node is visited

	flag = 0
	If(node_array(node,2) = 1)Then
			flag = 1
	End If

	IsVisited = flag
End Function


Function check_cell(mobot As Integer)
	Dim wall(4) As Integer 			'wall is described as (Right,Front,Left,Back)
	Dim s_val As Double				'sensor value
	Dim wall_threshold As Double	'min sensor measurement to determine if a wall exists


	'set initial wall thresh
	wall_threshold = 0.36


	'record initial position
	init_theta = GetMobotTheta(mobot)



	'-------------record R,F,L,B sides----------------
	For s = 0 To 2

		s_val = MeasureRange(mobot,s,0)
		If(s_val < wall_threshold And s_val <> -1)Then
			wall(s) = 1
		Else
			wall(s) = 0
		End If

	Next

	'
	'rotate90(mobot,"left")
	's_val = MeasureRange(mobot,2,0)
	'If(s_val < wall_threshold And s_val <> -1)Then
	'		wall(3) = 1
	'Else
	'		wall(3) = 0
	'End If


	'go back To initial position
	'While(GetMobotTheta(mobot) <> init_theta)
	'	rotate90(mobot,"left")
	'Wend

	check_cell = wall

End Function


Function move_to_end(mobot As Integer)

	'move forward 1 cell
	s_f = MeasureRange(mobot,1,0)

	While(s_f > 0.3 Or s_f = -1)

		mv_frwd_1cell(mobot)
		s_f = MeasureRange(mobot,1,0)
		MsgBox(CStr(s_f))
	Wend

End Function

Function mv_frwd_1cell(mobot As Integer)
	'Note:ssume that mobot will start in the middle of the cell
	'Note: assume each cell is 1 meter

	r = GetWheelDiameter(mobot)/2
	pi = 3.14159265359


	ts = 50 'seconds to get to othercell

	v = 1/ts		'velocities
	w = v/r

	rpm = w * (60/(2*pi))	'convert angular velocity to rpm
	SetWheelSpeed(mobot,rpm,rpm)

	For t = 1 To ts
		StepForward()
	Next

End Function


Function rotate90(mobot As Integer, dr As String)

	Dim pi As Double
	pi = 3.14159265359
	r = GetWheelDiameter(mobot)/2
	l = GetWheelsDistance(mobot)
	ts = 10  		'time to turn

	d = (pi/2) * (l/2)  'arc length to be covered

	v = d/ts		'velocities
	w = v/r

	rpm = w * (60/(2*pi))	'convert angular velocity to rpm

						 'drive wheels accordingly
	If(dr = "right")Then
		SetWheelSpeed(mobot,rpm,-rpm)
	ElseIf(dr = "left") Then
		SetWheelSpeed(mobot,-rpm,rpm)
	End If

	'running simulation
	For t = 1 To ts
		StepForward()
	Next

End Function