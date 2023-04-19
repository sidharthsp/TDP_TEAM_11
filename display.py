from controller import Supervisor, Display

display_length = 55
display_width = 40
map_length = 11
map_width = 8

def draw_football_court(display):
    width = 64
    height = 64
    green_colour = 0x3CB371
    display.setColor(green_colour)
    display.fillRectangle(0, 0, width, height)
    display.setColor(0xFFFFFF)
    # Draw the outer boundary
    display.setColor(0xFFFFFF)  # 白色
    display.drawRectangle(9, 17, 45, 30)  # 边线框
    display.drawRectangle(9, 19.5, 10, 26)  # 左禁区框
    display.drawRectangle(9, 24.5, 6, 16)  # 左小禁区框
    display.drawRectangle(6, 25.5, 4, 14)  # 左球门框
    display.drawRectangle(44, 19.5, 10, 26)  # 右禁区框
    display.drawRectangle(48, 24.5, 6, 16)  # 右小禁区框
    display.drawRectangle(53, 25.5, 4, 14)  # 右球门框

    # Draw the center circle
    display.drawOval(32, 32, 4, 4)

    # Draw the center line
    display.drawLine(32, 17, 32, 46)


def get_position_2d(node):
    position_3d = node.getPosition()
    return position_3d[0], position_3d[1]

def normalize_position(pos):
    x = (pos[1] + map_length / 2) / map_length * display_length + 4
    z = (pos[0] + map_width / 2) / map_width * display_width + 13
    # print(x)
    # print(z)
    return int(x), int(z)

def draw_node(display, football_pos, player1_pos, player2_pos,player3_pos,player4_pos
                        ,player5_pos,player6_pos,player7_pos,player8_pos):
    # Normalize the positions to fit within the Display dimensions


    football_pos_normalized = normalize_position(football_pos)
    player1_pos_normalized = normalize_position(player1_pos)
    player2_pos_normalized = normalize_position(player2_pos)
    player3_pos_normalized = normalize_position(player3_pos)
    player4_pos_normalized = normalize_position(player4_pos)
    player5_pos_normalized = normalize_position(player5_pos)
    player6_pos_normalized = normalize_position(player6_pos)
    player7_pos_normalized = normalize_position(player7_pos)
    player8_pos_normalized = normalize_position(player8_pos)

    # Draw the football and players
    display.setColor(0xCD5C5C)  # red
    display.fillOval(player1_pos_normalized[0], player1_pos_normalized[1], 2, 2)
    display.fillOval(player2_pos_normalized[0], player2_pos_normalized[1], 2, 2)
    display.fillOval(player3_pos_normalized[0], player3_pos_normalized[1], 2, 2)
    display.fillOval(player4_pos_normalized[0], player4_pos_normalized[1], 2, 2)
    display.setColor(0x0000CD)  # blue
    display.fillOval(player5_pos_normalized[0], player5_pos_normalized[1], 2, 2)
    display.fillOval(player6_pos_normalized[0], player6_pos_normalized[1], 2, 2)
    display.fillOval(player7_pos_normalized[0], player7_pos_normalized[1], 2, 2)
    display.fillOval(player8_pos_normalized[0], player8_pos_normalized[1], 2, 2)
    display.setColor(0xFFFF00)  # yellow
    display.fillOval(football_pos_normalized[0], football_pos_normalized[1], 1, 1)
    return

supervisor = Supervisor()
display = Display('display')

time_step = int(supervisor.getBasicTimeStep())
football_node = supervisor.getFromDef("ball")
player1_node = supervisor.getFromDef("R0")
player2_node = supervisor.getFromDef("R1")
player3_node = supervisor.getFromDef("R2")
player4_node = supervisor.getFromDef("R3")
player5_node = supervisor.getFromDef("B0")
player6_node = supervisor.getFromDef("B1")
player7_node = supervisor.getFromDef("B2")
player8_node = supervisor.getFromDef("B3")

while supervisor.step(time_step) != -1:
    football_pos = get_position_2d(football_node)
    player1_pos = get_position_2d(player1_node)
    player2_pos = get_position_2d(player2_node)
    player3_pos = get_position_2d(player3_node)
    player4_pos = get_position_2d(player4_node)
    player5_pos = get_position_2d(player5_node)
    player6_pos = get_position_2d(player6_node)
    player7_pos = get_position_2d(player7_node)
    player8_pos = get_position_2d(player8_node)
    draw_football_court(display)
    draw_node(display, football_pos, player1_pos, player2_pos,player3_pos,player4_pos
                        ,player5_pos,player6_pos,player7_pos,player8_pos)
