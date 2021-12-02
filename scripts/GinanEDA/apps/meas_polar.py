import numpy as np
from dash import html, dcc
from dash.dependencies import Input, Output, State
import plotly.express as px
import plotly.graph_objs as go

from app import app

from datasets import db

possible_plot = ['Line', 'Scatter']
dropdown_type = html.Div([
        dcc.Dropdown(
            id='mespolar_dropdown_type',
            options=[{'label': i, 'value': i} for i in possible_plot],
            placeholder="Graph Type",
            value=None
        )
    ],
    style={'width': '10%', 'display': 'inline-block'}
)


def dropdown_site(site_list):
    site_list2 = list(['ALL'])
    site_list2.extend(site_list)
    # print(site_list2)
    return html.Div([
        dcc.Dropdown(
            id='mespolar_dropdown_site',
            options=[{'label': i, 'value': i} for i in site_list2],
            placeholder = "Site",
            value=None
        )
        ],
        style={'width': '20%', 'display': 'inline-block'}
        )


def dropdown_sat(sat_list):
    sat_list2 = list(['ALL'])
    sat_list2.extend(sat_list)
    return html.Div([
        dcc.Dropdown(
            id='mespolar_dropdown_sat',
            options=[{'label': i, 'value': i} for i in sat_list2],
            placeholder="Sat",
            value=None
        )
    ],
        style={'width': '20%', 'display': 'inline-block'}
    )


def keys():
    a = db.MONGO_CL["Measurements"].find_one()
    temp = list(a.keys())
    return [{'label': i, 'value': i} for i in temp if i[0] != '_']


def dropdown_key_y():
    return html.Div([
        dcc.Dropdown(
            id='mespolar_dropdown_key_y',
            options=[i for i in keys()],
            placeholder="Y axis",
            value=None
        )
    ],
        style={'width': '20%', 'display': 'inline-block'}
    )


def dropdown_key_x():
    # print(keys())
    return html.Div([
        dcc.Dropdown(
            id='mespolar_dropdown_key_C',
            options=[i for i in keys()],
            placeholder="C Axis",
            value=None
        )
    ],
        style={'width': '20%', 'display': 'inline-block'}
    )

update_button = html.Div([
    html.Button("update", id='update_graph_measP', n_clicks=0)], style={'width': '5%', 'display': 'inline-block'}
)


def get_empty_graph(message):
    emptiness = {
        "layout": {
            "xaxis": {
                "visible": False
            },
            "yaxis": {
                "visible": False
            },
            "annotations": [
                {
                    "text": message,
                    "xref": "paper",
                    "yref": "paper",
                    "showarrow": False,
                    "font": {
                        "size": 28
                    }
                }
            ]
        }
    }
    return emptiness





def generate_trace(graph_type, x, y, label):
    if graph_type =="Line" or graph_type =="Scatter":
        if (graph_type == "Line"):
            mode = "lines"
        else:
            mode = "markers"
        # print(label, mode, graph_type)
        trace = go.Scatter(x=x,  y=y, mode=mode, name=label)
    elif(graph_type =="POLAR"):
        trace = go.Scatterpolar(r=y, theta=x, mode="markers", name=label)
    return trace


@app.callback(
    Output('plot_polar', 'figure'),
    inputs=[Input('update_graph_measP', 'n_clicks')],
    state=[
        # State('mespolar_dropdown_type',  'value'),
        State('mespolar_dropdown_site',  'value'),
        State('mespolar_dropdown_sat',   'value'),
        State('mespolar_dropdown_key_C', 'value'),
        State('mespolar_dropdown_site', 'options'),
        State('mespolar_dropdown_sat', 'options'),
    ])
def update_graph_measurements(click,  site, sat, caxis, list_site, list_sat):

    site = [i['value'] for i in list_site] if site == 'ALL' else [site]
    sat = [i['value'] for i in list_sat] if sat == 'ALL' else [sat]
    # print("HELLO Cplot",  site, sat, caxis)
    fig = go.Figure()
    if site is None or sat is None or caxis is None:
        return get_empty_graph("Make sure a value for all the Dropdown Menu is selected")
    else:
        site_, sat_, x_, y_, z_ = db.get_series_xyz('Measurements', None, site, sat, 'Azimuth', 'Elevation', caxis)
        trace = []
        min_ = 0
        max_ = 0
        # print("looking for min max ")
        for z in z_:
            min_ = min(min_, z.min())
            max_ = max(max_, z.max())
        # print('... MIN, MAX', min_, max_)
        min_ = np.floor(min_)
        max_ = np.ceil(max_)
        for i in range(len(x_)):
            # print(i, ' out of ', len(x_))
            # x_[i] = x[i]*np.pi/180.0
            # y_[i] = y[i]*np_pi/180
            # trace.append(generate_trace(graph_type, x_[i], y_[i], f'{site_[i]}-{sat_[i]}'))
            trace.append( go.Scatterpolar(r=y_[i], theta=x_[i], mode="markers",
                                          marker=dict(
                                              size=3,
                                              colorscale='Viridis',
                                              color=z_[i],
                                              showscale=True,
                                            cmin = min_,
                                            cmax = max_)
                                          ))
        fig = go.Figure(data=trace)
        fig.update_layout(xaxis=dict(rangeslider=dict(visible=True)), yaxis=dict(fixedrange=False),
                          polar=dict(
                              radialaxis_tickfont_size=8,
                              angularaxis=dict(
                                  tickfont_size=8,
                                  rotation=90,
                                  direction="clockwise"
                              ),
                              radialaxis= dict(range=[90,0])
                          ))
        fig.layout.autosize = True

    return fig


def layout():
    if db.MONGO_CL == None:
        return html.Div([html.P("First you will need to select a DB in the Db Info menu")])
    else:
        return html.Div(
            [
                # dropdown_type,
                dropdown_site(db.DB_SITE),
                dropdown_sat(db.DB_SAT),
                dropdown_key_x(),
                # dropdown_key_y(),
                update_button,
                dcc.Graph(
                    id='plot_polar',
                    figure=get_empty_graph("select information first")
                )
            ]
        )