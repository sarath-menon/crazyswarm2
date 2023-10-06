# -*- coding: utf-8 -*-
"""
plotting a generic USD log
"""

# attidtue best: 22

import cfusdlog
import matplotlib.pyplot as plt
import os
import sys
import yaml
from matplotlib.backends.backend_pdf import PdfPages


def file_guard(pdf_path):
    msg = None
    if os.path.exists(pdf_path):
        msg = input("file already exists, overwrite? [y/n]: ")
        if msg == "n":
            print("exiting...")
            sys.exit(0)
        elif msg == "y":
            print("overwriting...")
            os.remove(pdf_path)
        else:
            print("invalid msg...")
            file_guard(pdf_path)

    return


def process_data(data, settings):
    # adjust time
    start_time = settings["start_time"]
    end_time = settings["end_time"]
    event = settings["event_name"]

    if start_time is None:
        start_time = data[event]['timestamp'][0]
    else:
        start_time = min(start_time * 1000, data[event]['timestamp'][0])
    print("start_time:", start_time)

    # convert units
    if settings["convert_units"]:
        for key, value in settings["convert_units"].items():
            data[event][key] = data[event][key] * value

    # define time vector
    t = (data[event]['timestamp'] - start_time) / 1000

    return t, data_usd

def create_figures(data_usd, settings, log_str):
    t, data_usd = process_data(data_usd, settings)

    # create a PDF to save the figures
    pdf_path =  os.path.join(settings["output_dir"], log_str) + ".pdf"
    print("output path: {}".format(pdf_path))

    # check if user wants to overwrite the report file
    file_guard(pdf_path)

    pdf_pages = PdfPages(pdf_path)

    # create the title page
    title_text_settings = f"Settings:\n"
    for setting in settings["title_settings"]:
        title_text_settings += f"    {setting}: {settings[setting]}\n"

    # read the parameters from the info file
    info_file = f"info/{log_str.replace('log', 'info').split('_')[0]}.yaml" 
    try:
        with open(info_file, "r") as f:
            info = yaml.safe_load(f)
    except FileNotFoundError:
        print(f"File not found: {info_file}")
        exit(1)

    title_text_parameters = f"Parameters:\n"
    for key, value in info.items():
        title_text_parameters += f"    {key}: {value}\n"

    text = f"%% Lee controller tuning %%\n"
    title_text = text + "\n" + title_text_settings + "\n" + title_text_parameters + "\n"
    fig = plt.figure(figsize=(5, 6))
    fig.text(0.1, 0, title_text, size=11)
    pdf_pages.savefig(fig)

    # create data plots
    figures_max = settings.get("figures_max", None)  # set to None to plot all figures
    figure_count = 0
    for k, (event_name, data) in enumerate(data_usd.items()):
        if event_name in settings["event_name"]:
            print("processing event: {} ({})".format(event_name, k))

            # create a new figure for each value in the data dictionary
            for figure_info in settings["figures"]:
                if figures_max is not None and figure_count >= figures_max:
                    break

                title = figure_info["title"]
                figure_type = figure_info["type"]
                x_label = figure_info["xlabel"]
                z_label = figure_info.get("zlabel", None)
                structure = figure_info["structure"]
                structure_length = len(structure)
                
                if figure_type == "2d":
                    fig, ax = plt.subplots(structure_length, 1)

                    if structure_length == 1:
                        ax = [ax]
                    
                    # iterate over every subplot
                    for i, obj in enumerate(structure):
                        for x, y_dict in obj.items():
                            y_data = y_dict["y_info"]["data"]
                            y_label = y_dict["y_info"]["label"]
                            for signal_name, signal_legend in y_data.items():
                                if x == "timestamp":
                                    ax[i].plot(t, data_usd[event_name][signal_name], label=signal_legend, linewidth=0.5)
                                    ax[i].set_xlabel(x_label)
                                    ax[i].set_ylabel(y_label)
                                    ax[i].legend(loc="lower left", fontsize=5)
                                    ax[i].grid(True)

                if figure_type == "3d":
                    fig = plt.figure()
                    ax = fig.add_subplot(projection='3d')
                    y_label = figure_info["ylabel"]
                    
                    # iterate over every subplot
                    for i, obj in enumerate(structure):
                        ax.plot(data_usd[event_name][obj[0]],
                                data_usd[event_name][obj[1]],
                                data_usd[event_name][obj[2]], 
                                label=obj[3], 
                                linewidth=0.5)
                        
                    ax.set_xlabel(x_label)
                    ax.set_ylabel(y_label)
                    ax.set_zlabel(z_label)
                    ax.legend(loc="lower left", fontsize=5)
                    ax.grid(True)

                fig.suptitle(title, fontsize=16)
                plt.tight_layout()
                
                # save the figure as a page in the PDF
                pdf_pages.savefig(fig)
                plt.close(fig)

                figure_count += 1
                status_text = ">>> created figure {}: {}".format(figure_count, title)
                print(status_text)

    pdf_pages.close()


if __name__ == "__main__":
    # change the current working directory to the directory of this file
    os.chdir(os.path.dirname(os.path.abspath(__file__)))

    # load the plot settings
    settings_file = "settings.yaml"
    with open(settings_file, 'r') as f:
        settings = yaml.load(f, Loader=yaml.FullLoader)

    # get the log number from the user
    # log_num = input("Enter the logging number: ")
    # log_str = f"log{log_num}"

    # automatically scan and process the logs

    # (1) do a scan of the directories logs, info, and reports to see what logs have not been plotted yet
    logs_dir = settings["data_dir"]
    info_dir = settings["info_dir"]
    reports_dir = settings["output_dir"]

    processed_logs = []
    for root, _, files in os.walk(reports_dir):
        for filename in files:
            if filename.startswith("log") and filename.endswith(".pdf"):
                # Extract the log number from the PDF filename
                log_str = filename.strip(".pdf")
                processed_logs.append(log_str)
                
    non_processed_logs = []
    for root, _, files in os.walk(logs_dir):
        for filename in files:
            if filename.startswith("log"):
                if filename not in processed_logs:
                    non_processed_logs.append(filename)    

    # (2) plot them all
    print("====================================")
    print("...logs to process:")
    non_processed_logs.sort()
    for log_str in non_processed_logs:
        print(log_str)
    print("====================================")
    ans = input("Proceed? [y/n]: ")
    if ans == "n":
        print("Exiting...")
        sys.exit(0)
    print("...processing logs")
    print("====================================")

    for log_str in non_processed_logs:
        # decode binary log data
        path = os.path.join(settings["data_dir"], log_str)
        data_usd = cfusdlog.decode(path)

        # create the figures
        print("...creating figures")
        create_figures(data_usd, settings, log_str)
        print("...done creating figures")