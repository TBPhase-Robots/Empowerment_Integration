"""
A gui for the empowerment simulation which also runs the experiment via a series of menus.

Menu IDS are:
  -1: quit
   0: start menu
  10: information menu 1  (Removed following feedback from beta testers May 2022)
  11: information menu 2  (Removed following feedback from beta testers May 2022)
  12: information menu 3  (Removed following feedback from beta testers May 2022)
  13: information menu 4  (Removed following feedback from beta testers May 2022)
  14: information menu 5  (Removed following feedback from beta testers May 2022)
  20: instructions menu
  30: tutorial start page
  31: tutorial part 1 instructions
  32: tutorial part 2 instructions
  33: tutorial part 3 instructions
  34: tutorial part 4 instructions
  35: tutorial complete
  40: experimental block 1
  50: experimental block 2
  60: participants details
  70: debrief             (Removed following feedback from beta testers May 2022)
  80: final consent form  (Removed following feedback from beta testers May 2022)
  90: trial start
  91: post trial questions 1 (sliders)
  92: post trial questions 2 (sliders)
"""

#----------------EXTERNAL MODULES-----------------
import pygame
import pygame_menu
import runSimulation as sim
import colours
import gspread
import model.MenuLog as MenuLog
import os
import sys
#-------------------------------------------------


# -----------------DIRECTORIES, PATHS AND CONFIGURATIONS-----------------------
# Set the configuration and results directories folders
CONFIG_DIR = "experiment_config_files/"
RESULTS_DIR = os.path.join(os.path.expanduser('~'), "OneDrive - University of Bristol", "Empowerment Results")

# Credentials needed to log to a google sheet
#   This function was removed at the beta stage
# from oauth2client.service_account import ServiceAccountCredentials
# GOOGLE_SHEET_NAME = "empowerment_data" # save_details() has been removed
# CREDENTIALS_FILE_PATH = 'halogen-order-334818-44181576e631.json' save_details() has been removed

# Set the configuration files to present to the participant
LIVETEST_SEQUENCE_A = [
    'config_exp_1', 'config_exp_2', 'config_exp_3',
    'config_exp_4', 'config_exp_5', 'config_exp_6',
    'config_exp_7', 'config_exp_8', 'config_exp_9',
    'config_exp_10', 'config_exp_11', 'config_exp_12',
]
LIVETEST_SEQUENCE_B = [
    'config_exp_1', 'config_exp_2', 'config_exp_3',
    'config_exp_4', 'config_exp_5', 'config_exp_6',
    'config_exp_7', 'config_exp_8', 'config_exp_9',
    'config_exp_10', 'config_exp_11', 'config_exp_12',
]
TUTORIAL_SEQUENCE_A = ['config_fam_1', 'config_fam_2', 'config_fam_3', 'config_fam_4']

# Add config directory to all config files:
LIVETEST_SEQUENCE_A = [string for string in LIVETEST_SEQUENCE_A]
LIVETEST_SEQUENCE_B = [string for string in LIVETEST_SEQUENCE_B]
TUTORIAL_SEQUENCE_A = [string for string in TUTORIAL_SEQUENCE_A]

# Add folder to system path so config modules can be found and .
sys.path.append(CONFIG_DIR)
# ------------------------------------------------------------------


# --------------GLOBAL VARIABLES------------
# These initial values shouldn't be changed unless you know what you're doing!!
DEBUG_MODE_B = False
SCREEN_RESOLUTION = (1280, 720)
menu_screen = pygame.display.set_mode((1280, 720))
current_menu_id = 1
session_id = '000000T000000'

# tracks which test in the sequence is being undertaken
test_number = 0

# logging
menu_log = []
last_config_run = 'none'

# font sizes
title_size = 28
text_size = 22
max_char = 110
button_size = 28
our_theme = pygame_menu.themes.THEME_SOLARIZED

# --------------FUNCTIONS------------
def generate_session_id():
    global session_id
    from datetime import datetime
    session_id = datetime.now().strftime("%Y%m%dT%H%M%S")
#end function


def create_start_screen(full_screen_b=False):
    # Setup the screen 
    if full_screen_b:
        return pygame.display.set_mode((0, 0))
    else:
        return pygame.display.set_mode(SCREEN_RESOLUTION)
#end function


def set_difficulty(value, difficulty):
    # Do the job here !
    pass
#end function


def set_menu_id(menu_id, menu=[], save_details_b=False):
    """"
    Handles the change of one menu to the next.  

    menu_id: the ID of the menu to change to
    menu: the instance of the current menu
    save_details_b: if True then any user completed fields on the current menu will be logged
    """
    global current_menu_id
    global menu_log
    global last_config_run

    # If the current menu is directly related to a simulation then keep a record of the config name used in that simulation
    if current_menu_id == 91 or current_menu_id == 92:
        current_config = last_config_run
    else:
        current_config = 'none'

    # If true then pass the calling menu through a function which extracts any user inputs and stores them in a format for easy recall and logging
    #   The menu ID MUST be known to menu_log
    if save_details_b:
        menu_log.save_responses(menu, current_menu_id, current_config)

    # Set the current menu id which will cause the menu rendering function to change which menu is displayed
    current_menu_id = menu_id

    # -1 is a special menu which causes the program to exit cleanly
    if menu_id == -1:
        saveAndQuit()   
    return
#end function


def run_simulation(exit_to_menu, config_file_name='', list_of_configs=[], show_empowerment=True, use_task_weighted_empowerment=False):
    """
    Run's a single trial of the empowerment simulation

    """
    global test_number
    global session_id
    global menu_screen
    global current_menu_id
    global last_config_run

    # Menu callbacks aren't dynamic so need to do some tricks if the config file name isn't supplied but a list is
    if config_file_name == '' and not list_of_configs:
        raise Exception("list of configs can't be blank if no config file name provided!!")
    elif config_file_name == '':
        config_file_name = list_of_configs[test_number]

    if "_fam_" in config_file_name:
        show_empowerment = False

    # create a name for the config which includes information on the dynamic parameters
    #   i.e. those which are set at runtime rather than written in the config file
    #   These are useful for naming log files!
    config_name_with_parameters = config_file_name
    if show_empowerment:
        config_name_with_parameters = config_name_with_parameters + "_empshown"

    if use_task_weighted_empowerment:
        config_name_with_parameters = config_name_with_parameters + "_taskweighted"

    print(f'running a simulation with config {config_file_name} and #test {test_number}, exiting to menu {exit_to_menu}')
    if not DEBUG_MODE_B:
        sim.main(config_file_name, show_empowerment, use_task_weighted_empowerment, sim_session_id=session_id, log_file_name=config_name_with_parameters)
    
    # last_config_run is picked up by set_menu_id(...) and the menu logging system to log the slider responses
    #   the name needs to be unique and traceable to the parameter settings used for the simulation
    last_config_run = config_name_with_parameters

    # Not sure if this is always needed but to be on the safe side, recreate the menu window after the simulation window closes.
    menu_screen = create_start_screen()

    # set the menu we go to after the simulation is complete based on the parameter passed when the simulation was initiated
    current_menu_id = exit_to_menu
    
    # increment the number of tests completed
    test_number += 1
    return
#end function

def start_menu_setup():
    # create the menu
    global current_menu_id
    menu = pygame_menu.Menu('Welcome', 400, 500,
                        theme=our_theme)
    menu.add.button('Start', set_menu_id, 20, border_width=2)  # this is the information section, just renamed the button as START for congruency with the QUIT button
    # menu.add.button('Instructions', set_menu_id, 20)
    # menu.add.button('Enter Details', set_menu_id, 40)
    if DEBUG_MODE_B:
        menu.add.button('Tutorial (debug)', run_tutorial)
        menu.add.button('Play (debug)', run_simulation, 0)
    # menu.add.button ('Final Consent', set_menu_id, 50)
    # menu.add.button('Export Details', export_details, "live")
    # menu.add.button('Quit',  pygame_menu.events.EXIT)
    return menu
#end function

def instructions_menu_setup():
    global menu_screen
    title1 = ("Please read the instructions carefully\n")

    text1 = ("In this experiment, you will be part of a *team of virtual sheepdogs*,\nworking together to herd a flock of virtual sheep towards a safe area.\n"

             "The sheep are the *black* dots and the safe area is the *red square*.\n"
             "The dogs are *coloured dots*. Some dogs are already in the field.\n"
             "More dogs are available in a square *doghouse* to the right.\n"

             "The task must be completed *quickly* and *efficiently*, i.e., using the *minimum number of dogs*.\n \n"
             "You will complete *two* blocks of 12 trials.\nWhen each trial starts, some dogs will try to move the sheep towards the safe area.\n"

              "You will help by using your mouse buttons to *add* or *remove* dogs at any point.\n"
              "-- *Adding* dogs may help your team to complete the task more *quickly* --\n"
              "-- *Removing* dogs may help your team to complete the task more *efficiently* --\n\n"

              "For some trials it may not be possible to move all sheep into the safe area in the time available.\nIn such cases, please try to move them *as close to the safe area as possible*.\n\n"

              "At the end of each trial, you will be asked to answer a few short questions.\n"
              "Please answer these question promptly and move on to the next trial quickly.\n\n"
              "Before the experiment starts, a tutorial will explain how the trials work.\n")
    SCREEN_W, SCREEN_H = menu_screen.get_size()
    BORDER = 20
    menu = pygame_menu.Menu('Experiment Instructions', SCREEN_W - BORDER, SCREEN_H - BORDER, theme=our_theme)
    menu.add.label(title1, max_char=max_char, font_size=title_size, align=pygame_menu.locals.ALIGN_LEFT)
    menu.add.label(text1, max_char=max_char, font_size=text_size)  # , align=pygame_menu.locals.ALIGN_LEFT)
    # menu.add.button('Ok', set_menu_id, 30,font_size=20)
    menu.add.button('Ok', run_tutorial, font_size=button_size)
    return menu
#end function


def tutorial_start_menu_setup():
    """ Generates a menu for explaning and starting the tutorial """
    global menu_screen
    title = "Herding Tutorial"
    text = ("This tutorial will help you become familiar with the sheep herding task.\n\n"
            "The tutorial is arranged into 4 parts.\n"
            "You can repeat the tutorial until you are comfortable with the task.\n\n"
            "Press 'Ok' to Start. \n")
    SCREEN_W, SCREEN_H = menu_screen.get_size()
    BORDER = 20
    menu = pygame_menu.Menu(title, SCREEN_W - BORDER, SCREEN_H - BORDER, theme=our_theme)
    menu.add.label(text, max_char=max_char, font_size=title_size)
    menu.add.button('Ok', set_menu_id, 31, font_size=button_size)
    return menu


def tutorial_part1_setup():
    """ Generates a menu for the first part of the tutorial """
    global menu_screen
    title = "Tutorial Part 1: Adding a Dog from the Kennel"
    text = ("In Part 1, you will see *one* sheep outside the safe area.\n\n"
            "Use your *left* mouse button to *add* a dog from the doghouse to the field.\n"
            "(A dog will appear near the location of your cursor when you click.)\n"
            "Watch it move the sheep into the safe area.\n\n"
            "Press 'Ok' to Start Part 1. \n")
    SCREEN_W, SCREEN_H = menu_screen.get_size()
    BORDER = 20
    menu = pygame_menu.Menu(title, SCREEN_W - BORDER, SCREEN_H - BORDER, theme=our_theme)
    menu.add.label(text, max_char=max_char, font_size=title_size)
    menu.add.button('OK', run_simulation, 32, TUTORIAL_SEQUENCE_A[0], font_size=button_size)
    # menu.add.button('Start', run_tutorial)
    if DEBUG_MODE_B:
        menu.add.button('Skip (Debug)', set_menu_id, 32)
        menu.add.button('Main Menu (Debug)', set_menu_id, 0)
    return menu
#end function


def tutorial_part2_setup():
    """ Generates a menu for the second part of the tutorial """
    global menu_screen
    title = "Tutorial Part 2: Removing Dogs to the Kennel"
    text = ("In Part 2, you will see *one* sheep being herded by *six* dogs.\n\n"
            "Use your *right* mouse button to *remove* some dogs to the doghouse.\n"
            "(A dog near the location of your cursor will disappear when you click.)\n"
            "Watch the remaining team complete the herding task *more efficiently*.\n\n"
            "Press 'Ok' to Start Part 2. \n")
    SCREEN_W, SCREEN_H = menu_screen.get_size()
    BORDER = 20
    menu = pygame_menu.Menu(title, SCREEN_W - BORDER, SCREEN_H - BORDER, theme=our_theme)
    menu.add.label(text, max_char=max_char, font_size=title_size)
    menu.add.button('OK', run_simulation, 33, TUTORIAL_SEQUENCE_A[1], font_size=button_size)
    # menu.add.button('Start', run_tutorial)
    if DEBUG_MODE_B:
        menu.add.button('Skip (Debug)', set_menu_id, 33)
        menu.add.button('Main Menu (Debug)', set_menu_id, 0)
    return menu
#end function


def tutorial_part3_setup():
    """ Generates a menu for the third part of the tutorial """
    global menu_screen
    title = "Tutorial Part 3: Speed"
    text = ("In Part 3, your team is *struggling* to herd a flock of sheep.\n\n"
            "It will take a long time for your team to complete the task.\n"
            "Adding one or two dogs will help the team to work more *quickly*.\n"
            "Completing the task more quickly will *increase* your team's performance score.\n \n"
            "Press 'Ok' to start Part 3 (or you can choose to repeat Parts 1 and 2)\n")
    SCREEN_W, SCREEN_H = menu_screen.get_size()
    BORDER = 20
    menu = pygame_menu.Menu(title, SCREEN_W - BORDER, SCREEN_H - BORDER, theme=our_theme)
    menu.add.label(text, max_char=max_char, font_size=title_size)
    menu.add.button('OK', run_simulation, 34, TUTORIAL_SEQUENCE_A[2], font_size=button_size)
    menu.add.button('Repeat Parts 1 and 2', set_menu_id, 31)
    # menu.add.button('Start', run_tutorial)
    if DEBUG_MODE_B:
        menu.add.button('Skip', set_menu_id, 32)
        menu.add.button('Main Menu', set_menu_id, 0)
    return menu
#end function


def tutorial_part4_setup():
    """ Generates a menu for the forth part of the tutorial """
    global menu_screen
    global session_id
    title = "Tutorial Part 4: Efficiency"
    text = ("In Part 4, your team is *easily* herding a flock of sheep.\n\n"
            "There are more than enough dogs in the field. Some are not needed.\n"
            "Removing one or two dogs will help the team to work more *efficiently*.\n"
            "Completing the task with fewer dogs will *increase* your team's performance score.\n\n"
            "Press 'Ok' to Start Part 4. \n")
    SCREEN_W, SCREEN_H = menu_screen.get_size()
    BORDER = 20
    menu = pygame_menu.Menu(title, SCREEN_W - BORDER, SCREEN_H - BORDER, theme=our_theme)
    menu.add.label(text, max_char=max_char, font_size=title_size)
    menu.add.button('OK', run_simulation, 35, TUTORIAL_SEQUENCE_A[3], font_size=button_size)
    # menu.add.button('Start', run_tutorial)
    if DEBUG_MODE_B:
        menu.add.button('Skip (Debug)', set_menu_id, 33)
        menu.add.button('Main Menu (Debug)', set_menu_id, 0)
    return menu
#end function   


def tutorial_complete_setup():
    """ Generates a menu for after the final part of the tutorial has been completed """
    global menu_screen
    title = "Tutorial Complete"
    text = ("Congratulations, you've completed the tutorial!\n\n"
            "Press 'Continue' to start the experiment.\n"
            "Or, you can repeat some or all of the Tutorial. \n")
    SCREEN_W, SCREEN_H = menu_screen.get_size()
    BORDER = 20
    menu = pygame_menu.Menu(title, SCREEN_W - BORDER, SCREEN_H - BORDER, theme=our_theme)
    menu.add.label(text, max_char=max_char, font_size=title_size)
    menu.add.button('Continue to the Experiment', run_experiment)
    menu.add.button('Repeat Parts 3 and 4 of the Tutorial', set_menu_id, 33)
    menu.add.button('Repeat the Whole Tutorial', set_menu_id, 31)
    # menu.add.button('Repeat', set_menu_id, 30)
    return menu
#end function


### Here, the experiment should start. Specific instructions, depending on the block need to be added. Since there is no section dedicated to each block
### i have created two sections in which instructions are displayed for each block. Parcicipants could either press 'Start' to start the experiment
### or press 'Back' to return to the familiarisation trials
### I also added a 'quit' button. What is needed here, is to load the simulation

def experimental_block_1_setup():
    global menu_screen
    # show the third screen
    title = "Experiment Block"
    text = ("You will now be presented with a block of trials.\n\n"

            "Your task is to help your team of dogs (blue dots)\nherd the sheep (black dots) into the safe area (the red square).\n\n"

            "Use your mouse to *add* (left click) or *remove* (right click) dogs\nto complete the task quickly *and* efficiently.\n\n"

            "Remember: high performance is achieved by moving all sheep towards the safe area\nas quickly as possible *and* as efficiently as possible,\ni.e., herd *quickly*, but use the *minimum number of dogs*.\n\n"

            "Press 'Continue' to start the block.\n")
            # "If you wish to repeat the tutorial, press 'Repeat Tutorial'\n")
    SCREEN_W, SCREEN_H = menu_screen.get_size()
    BORDER = 20
    menu = pygame_menu.Menu(title, SCREEN_W - BORDER, SCREEN_H - BORDER, theme=our_theme)
    menu.add.label(text, max_char=max_char, font_size=title_size)
    menu.add.button('Continue', set_menu_id, 90)  # at the moment, this goes traight to block 2 as I am not able to link it to the simulation.
    # menu.add.button('Repeat Tutorial', run_tutorial)
    return menu
#end function


def experimental_block_2_setup():
    global menu_screen
    # show the third screen
    title = "Experimental Block"
    text = ("You will now be presented with a block of trials.\n\n"
            "Your task is to help your team of dogs (coloured dots)\nherd the sheep (black dots) into the safe area (the red square).\n\n"

            "The changing colour of each dog indicates how much influence\nthat dog feels that it has at that moment:\n"
            "Red = little influence on the world around it\n"
            "Orange = moderate influence on the world around it\n"
            "Green = strong influence on the world around it\n\n"

            "Use your mouse to *add* (left click) or *remove* (right click) dogs\nto complete the task quickly and efficiently.\n\n"

            "Remember: high performance is achieved by moving all sheep towards the safe area\nas quickly as possible *and* as efficiently as possible,\ni.e., herd *quickly*, but use the *minimum number of dogs*.\n\n"

            "Press 'Continue' to start the block.\n")

    SCREEN_W, SCREEN_H = menu_screen.get_size()
    BORDER = 20
    menu = pygame_menu.Menu(title, SCREEN_W - BORDER, SCREEN_H - BORDER, theme=our_theme)
    menu.add.label(text, max_char=max_char, font_size=title_size)
    menu.add.button('Continue', set_menu_id, 90)  # at the moment, the code goes straight to the final consent.
    if DEBUG_MODE_B:
        menu.add.button('Quit (Debug)', set_menu_id, -1, menu, True)  # Alternatively, given that this is the second block, participants can decide just to quit the experiment.
    return menu
#end function

def details_setup():
    global menu_screen

    SCREEN_W, SCREEN_H = menu_screen.get_size()
    BORDER = 20

    title = "Final Questions"
    text = ("Please answer the following final questions and then press Finish\n")

    menu = pygame_menu.Menu(title, SCREEN_W - BORDER, SCREEN_H - BORDER, theme=our_theme)
    menu.add.label(text, max_char=max_char, font_size=title_size)

    menu.add.text_input('Date of Birth (dd/mm/yyyy):  ', default='', textinput_id='birth', input_underline='_', input_underline_len=12)
    menu.add.text_input('Sex:  ', default='', textinput_id='sex', input_underline='_', input_underline_len=15)
    #menu.add.text_input('Is English your first language? (Y/N):  ', default='', textinput_id='english', input_underline='_', input_underline_len=0)
    menu.add.dropselect(title='Is English your first language?', items=[('Yes' ,0), ('No', 1)], dropselect_id = 'english', font_size=title_size, selection_option_font_size=title_size-2)
    #menu.add.text_input('Do you have normal / corrected-to-normal vision? (Y/N):  ', default='', textinput_id='vision', input_underline='_', input_underline_len=0))
    menu.add.dropselect(title='Do you have normal / corrected-to-normal vision?', items=[('Yes', 0), ('No', 1)], dropselect_id = 'vision', font_size=title_size, selection_option_font_size=title_size-2)
    #menu.add.text_input('Do you have colour blindness or a colour vision deficiency? (Y/N):   ', default='', textinput_id='colour')
    menu.add.dropselect(title='Do you have colour blindness or a colour vision deficiency?', items=[('Yes', 0), ('No', 1)], dropselect_id = 'colour', font_size=title_size, selection_option_font_size=title_size-2)
    
    #menu.add.button('Finish', set_menu_id, 70, menu, True)
    menu.add.button('Finish',  set_menu_id, -1, menu, True)
    return menu
#end function

def test_start_setup(list_of_configs, show_empowerment, use_taskweighted_empowerment):
    global menu_screen
    global test_number
    # show the third screen
    title = "Experiment Trial"
    text = ("Press Go when you are ready to start the next trial.")
    SCREEN_W, SCREEN_H = menu_screen.get_size()
    BORDER = 20
    menu = pygame_menu.Menu(title, SCREEN_W - BORDER, SCREEN_H - BORDER, theme=our_theme)
    menu.add.label(text, max_char=max_char, font_size=28)
    menu.add.button('Go', run_simulation, 91, '', list_of_configs, show_empowerment, use_taskweighted_empowerment)
    if DEBUG_MODE_B:
        menu.add.button('Skip', set_menu_id, 0)
        menu.add.button('Main Menu', set_menu_id, 0)
    return menu
#end function


def post_test_questions_setup1():
    global menu_screen
    SCREEN_W, SCREEN_H = menu_screen.get_size()
    BORDER = 20
    import numpy as np
    SLIDER_VALUES = np.arange(0, 20.5, 0.5).tolist()
    menu = pygame_menu.Menu('Well Done!', SCREEN_W - BORDER, SCREEN_H - BORDER, theme=our_theme)
    menu.add.label('Please answer the following question promptly...\n', max_char=max_char, font_size=title_size)
    menu.add.label('Use the slider to indicate how many seconds elapsed\nbetween your final mouse click and the end of the trial:\n', max_char=max_char, font_size=title_size)

    menu.add.range_slider('', default=10, range_values=SLIDER_VALUES, increment=0.5, rangeslider_id='time', width=500, range_line_height=10, 
        range_text_value_color=(255, 0, 125), range_text_value_enabled=False, range_text_value_tick_number=3)

    menu.add.button('Done', set_menu_id, 92, menu, True)
    # menu.add.button('Main Menu', set_menu_id, 0)
    return menu
#end function


def post_test_questions_setup2():
    global menu_screen
    SCREEN_W, SCREEN_H = menu_screen.get_size()
    BORDER = 20
    SLIDER_VALUES = {0: 'not at all', 1: '', 2: '', 3: '', 4: '', 5: '', 6: 'completely'}
    menu = pygame_menu.Menu('Well Done!', SCREEN_W - BORDER, SCREEN_H - BORDER, theme=our_theme)
    menu.add.label('Please answer the following two questions promptly...\n', max_char=max_char, font_size=title_size)
    
    menu.add.label('Regardless of how well you feel your team performed,\nto what extent did you feel **engaged in the task**?', max_char=max_char, font_size=title_size, underline=False)
    
    menu.add.label('From not at all (left) to completely (right)', max_char=max_char, font_size=text_size)
    menu.add.range_slider('', default=3, range_values=list(SLIDER_VALUES.keys()), increment=1, rangeslider_id='engaged', width=500, range_line_height=10, 
        range_text_value_color=(255, 0, 125), range_text_value_enabled=True, slider_text_value_enabled=False, value_format=lambda x: SLIDER_VALUES[x])
    
    menu.add.label('\nRegardless of how well you feel your team performed,\nto what extent did you feel *part of the team*?', max_char=max_char, font_size=title_size)
    menu.add.label('From not at all (left) to completely (right)', max_char=max_char, font_size=text_size)
    menu.add.range_slider('', default=3, range_values=list(SLIDER_VALUES.keys()), increment=1, rangeslider_id='part_of_team', width=500, range_line_height=10, 
        range_text_value_color=(255, 0, 125), range_text_value_enabled=True, slider_text_value_enabled=False, value_format=lambda x: SLIDER_VALUES[x])
    menu.add.button('Done', set_menu_id, 90, menu, True)
    # menu.add.button('Main Menu', set_menu_id, 0)
    return menu
#end function

def draw_menu_background():
    # fills the screen with black
    global menu_screen
    menu_screen.fill(colours.BLACK)
    return
#end function

def run_experimental_block(list_of_configs, show_empowerment, use_taskweighted):
    """"
    Runs a set of experiments. Dynamic switches to the parameter values used in the configurations are processed in here
    
    list_of_configs: the names of the configs to run in this block
    show_empowerment: if true then the dogs show their empowerment value to the user
    use_taskweighted: if true then the dogs use a task weighted measure of empowerment
    """

    global test_number
    global current_menu_id
    test_number = 0
    is_test_complete_b = False

    # Create and setup the menus
    experimental_block_1_setup_m = experimental_block_1_setup()
    experimental_block_2_setup_m = experimental_block_2_setup()
    test_start_setup_m = test_start_setup(list_of_configs, show_empowerment, use_taskweighted)
    post_test_questions_m1 = post_test_questions_setup1()
    post_test_questions_m2 = post_test_questions_setup2()

    # Loop until all the experiments given in list_of_configs have been run once
    while test_number <= len(list_of_configs) and not is_test_complete_b:
        # Blank the screen so old menu's aren't displayed behind new ones
        draw_menu_background()

        # Handle any keyboard or mouse inputs
        events = pygame.event.get()
        for event in events:
            if event.type == pygame.QUIT:
                exit()

        # A sneaky trick, if the menu is set to the start test screen but we've reached the end of the config list then the block of tests is complete
        #   and the exectution flow should switch back to the master function which is running the experiments.
        #   This only works because after a test is run it will go to the post test questions before returning to the start new test menu
        if current_menu_id == 90 and test_number == len(list_of_configs):
            is_test_complete_b = True
        
        elif current_menu_id == 40:
            experimental_block_1_setup_m.update(events)
            experimental_block_1_setup_m.draw(menu_screen)
        
        elif current_menu_id == 50:
            experimental_block_2_setup_m.update(events)
            experimental_block_2_setup_m.draw(menu_screen)
        
        elif current_menu_id == 90:
            # These lines reset the sliders on the post question menus to their default values.  
            #   There may be a better way to do this via a call back.
            post_test_questions_m1.get_widget('time').reset_value()
            post_test_questions_m2.get_widget('engaged').reset_value()
            post_test_questions_m2.get_widget('part_of_team').reset_value()
            test_start_setup_m.update(events)
            test_start_setup_m.draw(menu_screen)
        
        elif current_menu_id == 91:
            post_test_questions_m1.update(events)
            post_test_questions_m1.draw(menu_screen)
       
        elif current_menu_id == 92:
            post_test_questions_m2.update(events)
            post_test_questions_m2.draw(menu_screen)
        
        else:
            # the menu is outside the list for the experimental block so mark as complete
            is_test_complete_b = True

        # display any changes to the menu
        pygame.display.update()

    # exiting this function should drop back to a master function which is running the experimental blocks
    return
#end function


def run_experiment():
    """
    Runs both the empowerment shown and not shown blocks of trials, 
    and it applies the dynamic configuration parameters to show/not show empowerment and use vanilla/taskweighted empowerment 
    """
    global menu_screen
    global current_menu_id
    import numpy as np

    # Initialise the dynamic parameters to be set later
    show_empowerment = None
    use_taskweighted_empowerment = None
    
    # determine the order of the experimental blocks
    num_of_dirs = len(os.listdir(RESULTS_DIR))

    if num_of_dirs % 2 == 0:
        block_order = (['empowerment_shown', 'no_empowerment_shown'])

    else:
        block_order = (['no_empowerment_shown', 'empowerment_shown'])

    if (num_of_dirs % 4) >= 2:
        use_taskweighted_empowerment = True
    else:
        use_taskweighted_empowerment = False

    # Run each block of trials
    for block in block_order:
        if block == 'empowerment_shown':
            # set which menu to start in
            current_menu_id = 50
            # determine the order to run the sequence
            config_order = np.random.permutation(len(LIVETEST_SEQUENCE_A))
            configs = np.array(LIVETEST_SEQUENCE_A)
            show_empowerment = True

        else:
            print("I'm running the empowerment NOT shown block")
            # set which menu to start in
            current_menu_id = 40
            # determine the order to run the sequence
            config_order = np.random.permutation(len(LIVETEST_SEQUENCE_B))
            configs = np.array(LIVETEST_SEQUENCE_B)
            show_empowerment = False

        # shuffle the test sequence order
        print(f"I'm running experiment block {block} with the config order {config_order}")
        configs = configs[config_order]
        run_experimental_block(configs.tolist(), show_empowerment, use_taskweighted_empowerment)

    # set the menu ID to the post experiment questions and debreif
    current_menu_id = 60
    # this should return to main()
    return
#end function


def saveAndQuit():
    """
    Saves everything logged from the menu's to file and closes the program
        This should be run in order to exit the program cleanly and not lose data!
    """
    global menu_log
    menu_log.pickleLog(os.path.join(RESULTS_DIR, session_id, ""))
    exit()
    return
#end function


def run_tutorial():
    """
    Runs the block of trials used in the tutorial
    """
    global menu_screen
    global current_menu_id

    print("I'm running the tutorial")

    # Create an initialise the menus
    tutorial_start_m = tutorial_start_menu_setup()
    tutorial_part1_m = tutorial_part1_setup()
    tutorial_part2_m = tutorial_part2_setup()
    tutorial_part3_m = tutorial_part3_setup()
    tutorial_part4_m = tutorial_part4_setup()
    tutorial_complete_m = tutorial_complete_setup()

    # Make sure the starting menu screen is tutorial start
    current_menu_id = 30
    running_tutorial_b = True

    while running_tutorial_b:
        # blank the screen so old menu's aren't displayed behind new ones
        draw_menu_background()

        events = pygame.event.get()
        for event in events:
            if event.type == pygame.QUIT:
                saveAndQuit()

        # select the right menu for the state, update it (check if buttons have been pushed etc) and draw.
        if current_menu_id == 30:
            tutorial_start_m.update(events)
            tutorial_start_m.draw(menu_screen)
        elif current_menu_id == 31:
            tutorial_part1_m.update(events)
            tutorial_part1_m.draw(menu_screen)
        elif current_menu_id == 32:
            tutorial_part2_m.update(events)
            tutorial_part2_m.draw(menu_screen)
        elif current_menu_id == 33:
            tutorial_part3_m.update(events)
            tutorial_part3_m.draw(menu_screen)
        elif current_menu_id == 34:
            tutorial_part4_m.update(events)
            tutorial_part4_m.draw(menu_screen)
        elif current_menu_id == 35:
            tutorial_complete_m.update(events)
            tutorial_complete_m.draw(menu_screen)
        else:
            running_tutorial_b = False

        pygame.display.update()
    return
#end function


def main():
    """
    Runs the top level menu structure which contains the main menu, briefing and debriefing screens.
    The program uses two sub menu structures which aren't handled in this function. These are:
     - run_tutorial: runs the menus used in the tutorial
     - run_experment: runs the menus used in the experment
    """
    global menu_screen
    global current_menu_id
    global menu_log
    global session_id

    # Create a unique session ID
    generate_session_id()

    # Cetup the logging for user responses entered on the menus
    menu_log = MenuLog.MenuLog(session_id)

    # Create and setup the pygame window/screen
    menu_screen = create_start_screen();  

    # Start pygame
    pygame.init()

    # Create and setup the menus
    start_m = start_menu_setup()
    # information_1_m = information_sheet1_setup()
    # information_2_m = information_sheet2_setup()
    # information_3_m = information_sheet3_setup()
    # information_4_m = information_sheet4_setup()
    # information_5_m = information_sheet5_setup()
    instructions_m = instructions_menu_setup()
    details_m = details_setup()
    # debrief_setup_m = debrief_setup()
    # personal_details_m = personal_details_setup()
    # final_consent_m = final_consent_setup()

    # Loop until the program exits
    while True:
        # Blank the screen so old menu's aren't displayed behind new ones
        draw_menu_background()

        # Handle the event queue
        events = pygame.event.get()
        for event in events:
            if event.type == pygame.QUIT:
                saveAndQuit()

        # Select the right menu for the program state, update it (check if buttons have been pushed etc) and draw.
        if current_menu_id == 0:
            start_m.update(events)
            start_m.draw(menu_screen)
        elif current_menu_id == 10:
            # information_1_m.update(events)
            # information_1_m.draw(menu_screen)
            pass
        elif current_menu_id == 11:
            # information_2_m.update(events)
            # information_2_m.draw(menu_screen)
            pass
        elif current_menu_id == 12:
            # information_3_m.update(events)
            # information_3_m.draw(menu_screen)
            pass
        elif current_menu_id == 13:
            # information_4_m.update(events)
            # information_4_m.draw(menu_screen)
            pass
        elif current_menu_id == 14:
            # information_5_m.update(events)
            # information_5_m.draw(menu_screen)
            pass
        elif current_menu_id == 20:
            instructions_m.update(events)
            instructions_m.draw(menu_screen)
        # menu_id 30-35 are handled in run_tutorial()
        # menu_id 40,50,90-92 are handled in run_experiment()
        elif current_menu_id == 60:
            details_m.update(events)
            details_m.draw(menu_screen)
        elif current_menu_id == 70:
            # debrief_setup_m.update(events)
            # debrief_setup_m.draw(menu_screen)
            pass
        elif current_menu_id == 80:
            # final_consent_m.update(events)
            # final_consent_m.draw(menu_screen)
            pass
        else:
            start_m.update(events)
            start_m.draw(menu_screen)

        pygame.display.update()
    #end while loop
    return
#end function


if __name__ == '__main__':
    main()