import matplotlib

matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2TkAgg
from matplotlib.figure import Figure

import Tkinter as tk
import ttk as ttk


class Backster(tk.Tk):
    def __init__(self, *args, **kwargs):
        tk.Tk.__init__(self, *args, **kwargs)

        tk.Tk.wm_title(self, "Sea of BTC client")

        container = tk.Frame(self)
        container.pack(side="top", fill="both", expand=True)
        container.grid_rowconfigure(0, weight=1)
        container.grid_columnconfigure(0, weight=1)

        self.frames = {}

        # initialize the two pages
        frame1 = StartPage(container, self)
        self.frames[StartPage] = frame1
        frame1.grid(row=0, column=0, sticky="nsew")

        frame2 = PlotPage(container, self, frame1)
        self.frames[PlotPage] = frame2
        frame2.grid(row=0, column=0, sticky="nsew")

        self.show_frame(StartPage)

    def show_frame(self, cont):
        frame = self.frames[cont]
        frame.tkraise()

    def end_screen(self):
        self.destroy()


class StartPage(tk.Frame):

    def __init__(self, parent, controller):
        tk.Frame.__init__(self, parent)

        # initialize variables
        self.l1 = tk.StringVar()
        self.l2 = tk.StringVar()
        self.l3 = tk.StringVar()
        self.l4 = tk.StringVar()
        self.l5 = tk.StringVar()
        self.length1 = tk.StringVar(value=0.0)
        self.length2 = tk.StringVar(value=0.0)
        self.length3 = tk.StringVar(value=0.0)
        self.length4 = tk.StringVar(value=0.0)
        self.length5 = tk.StringVar(value=0.0)
        self.exercise = tk.StringVar(value="Select Exercise")

        self.grid(column=0, row=0, sticky=('n', 'w', 'e', 's'))

        # create column 1 features
        ttk.Label(self, text="Exercise").grid(column=1, row=1, sticky='n')
        ttk.Button(self, text="Toe Touch", command=lambda: self.exercise_select(1)).grid(column=1, row=2, sticky='n')
        ttk.Button(self, text="Back Bend", command=lambda: self.exercise_select(2)).grid(column=1, row=3, sticky='n')
        ttk.Button(self, text="Torsional Twist", command=lambda: self.exercise_select(3)).grid(column=1, row=4, sticky='n')
        ttk.Label(self, textvariable=self.exercise).grid(column=1, row=5, sticky='s')

        # column 2
        ttk.Label(self, text="Link 1:").grid(column=2, row=2, sticky='e')
        ttk.Label(self, text="Link 2:").grid(column=2, row=3, sticky='e')
        ttk.Label(self, text="Link 3:").grid(column=2, row=4, sticky='e')
        ttk.Label(self, text="Link 4:").grid(column=2, row=5, sticky='e')
        ttk.Label(self, text="Link 5:").grid(column=2, row=6, sticky='e')

        # column 3
        ttk.Label(self, text="Link Lengths (cm)").grid(column=3, row=1, sticky='n')
        l1_entry = ttk.Entry(self, width=4, textvariable=self.l1)
        l1_entry.grid(column=3, row=2, sticky=('w', 'e'))
        l2_entry = ttk.Entry(self, width=4, textvariable=self.l2)
        l2_entry.grid(column=3, row=3, sticky=('w', 'e'))
        l3_entry = ttk.Entry(self, width=4, textvariable=self.l3)
        l3_entry.grid(column=3, row=4, sticky=('w', 'e'))
        l4_entry = ttk.Entry(self, width=4, textvariable=self.l4)
        l4_entry.grid(column=3, row=5, sticky=('w', 'e'))
        l5_entry = ttk.Entry(self, width=4, textvariable=self.l5)
        l5_entry.grid(column=3, row=6, sticky=('w', 'e'))
        ttk.Button(self, text="Set Lengths", command=self.setLengths).grid(column=3, row=7, sticky='w')

        # column 4
        ttk.Label(self, textvariable=self.length1).grid(column=4, row=2, sticky=('w', 'e'))
        ttk.Label(self, textvariable=self.length2).grid(column=4, row=3, sticky=('w', 'e'))
        ttk.Label(self, textvariable=self.length3).grid(column=4, row=4, sticky=('w', 'e'))
        ttk.Label(self, textvariable=self.length4).grid(column=4, row=5, sticky=('w', 'e'))
        ttk.Label(self, textvariable=self.length5).grid(column=4, row=6, sticky=('w', 'e'))
        ttk.Button(self, text="Start", command=lambda: self.start(self.exercise.get(), controller))\
            .grid(column=4, row=7, sticky='w')

        # make it look pretty
        for child in self.winfo_children(): child.grid_configure(padx=5, pady=5)
        # put cursor in the first box
        l1_entry.focus()
        # allow hitting enter to set Lengths
        self.bind('<Return>', self.setLengths)

    def start(self, ex, controller):
        self.setLengths()

        # initialize at 0
        link1 = 0.0
        link2 = 0.0
        link3 = 0.0
        link4 = 0.0
        link5 = 0.0

        if ex == "Select Exercise":
            return
        else:
            # set values in script to these
            try:
                link1 = float(self.l1.get())
                link2 = float(self.l2.get())
                link3 = float(self.l3.get())
                link4 = float(self.l4.get())
                link5 = float(self.l5.get())
            except ValueError:
                pass
                return

            # close window
            controller.show_frame(PlotPage)

        return link1, link2, link3, link4, link5, ex

    def setLengths(self, *args):
        try:
            value1 = float(self.l1.get())
            self.length1.set(value1)
        except ValueError:
            pass
        try:
            value2 = float(self.l2.get())
            self.length2.set(value2)
        except ValueError:
            pass
        try:
            value3 = float(self.l3.get())
            self.length3.set(value3)
        except ValueError:
            pass
        try:
            value4 = float(self.l4.get())
            self.length4.set(value4)
        except ValueError:
            pass
        try:
            value5 = float(self.l5.get())
            self.length5.set(value5)
        except ValueError:
            pass

    def exercise_select(self, enum):
        if enum == 1:
            self.exercise.set("Toe Touch")
        elif enum == 2:
            self.exercise.set("Back Bend")
        elif enum == 3:
            self.exercise.set("Twist")

        return


class PlotPage(tk.Frame, StartPage):
    def __init__(self, parent, controller, start_page):
        tk.Frame.__init__(self, parent)

        # create variables to clean things up
        self.l1 = float(start_page.length1.get())
        self.l2 = float(start_page.length2.get())
        self.l3 = float(start_page.length3.get())
        self.l4 = float(start_page.length4.get())
        self.l5 = float(start_page.length5.get())

        button1 = ttk.Button(self, text="End Session",
                             command=lambda: controller.end_screen())
        button1.pack()

        f = Figure(figsize=(7, 5), dpi=100)
        a = f.add_subplot(111)
        a.plot([1, 2, 3, 4, 5, 6, 7, 8], [self.l1, self.l2, self.l3, self.l4, self.l5, 1, 1, 1])

        canvas = FigureCanvasTkAgg(f, self)
        canvas.show()
        canvas.get_tk_widget().pack(side=tk.BOTTOM, fill=tk.BOTH, expand=True)

        toolbar = NavigationToolbar2TkAgg(canvas, self)
        toolbar.update()
        canvas._tkcanvas.pack(side=tk.TOP, fill=tk.BOTH, expand=True)




app = Backster()
app.mainloop()
