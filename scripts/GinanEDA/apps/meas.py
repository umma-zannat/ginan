from dash import html, dcc
from dash.dependencies import Input, Output, State
import plotly.express as px
import plotly.graph_objs as go

from app import app

from datasets import db

possible_plot = ['Line', 'Scatter', 'Polar']
dropdown_type = html.Div([
        dcc.Dropdown(
            id='mes_dropdown_type',
            options=[{'label': i, 'value': i} for i in possible_plot],
            placeholder="Graph Type",
            value=None
        )
    ],
    style={'width': '10%', 'display': 'inline-block'}
)


def exclude_start():
    return html.Div([
        html.P('Exclude the first points',style={'display':'inline-block','margin-right':20}),
        dcc.Input(id='exclude_npt', value='0', type='text', size='2'),
        html.P(' points',style={'display':'inline-block','margin-right':5}),
    ])

def dropdown_site(site_list):
    site_list2 = list(['ALL'])
    site_list2.extend(site_list)
    # print(site_list2)
    return html.Div([
        dcc.Dropdown(
            id='mes_dropdown_site',
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
            id='mes_dropdown_sat',
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
            id='mes_dropdown_key_y',
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
            id='mes_dropdown_key_x',
            options=[i for i in keys()],
            placeholder = "X Axis",
            value=None
        )
    ],
        style={'width': '20%', 'display': 'inline-block'}
    )

update_button = html.Div([
    html.Button("update graph", id='update_graph', n_clicks=0)], style={'width': '5%', 'display': 'inline-block'}
)


def get_empty_graph(message):
    emptiness = {
        "layout": {
            "xaxis": {
                "visible": False
            },
            "yaxis": {
                "visible": False,
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
    elif(graph_type =="Polar"):
        trace = go.Scatterpolar(r=y, theta=x, mode="markers", name=label)
    return trace


@app.callback(
    Output('plot1', 'figure'),
    inputs=[Input('update_graph', 'n_clicks')],
    state=[
        State('mes_dropdown_type',  'value'),
        State('mes_dropdown_site',  'value'),
        State('mes_dropdown_sat',   'value'),
        State('mes_dropdown_key_x', 'value'),
        State('mes_dropdown_key_y', 'value'),
        State('mes_dropdown_site', 'options'),
        State('mes_dropdown_sat', 'options'),
        State('exclude_npt', 'value')
    ])
def update_graph_measurements(click, graph_type, site, sat, xaxis, yaxis, list_site, list_sat, exclude):
    # print("HELLO", graph_type, site, sat, xaxis, yaxis)
    try:
        exclude = int(exclude)
    except:
        exclude = 0

    if exclude < 0:
        exclude = 0

    fig = go.Figure()
    site = [i['value'] for i in list_site] if site == 'ALL' else [site]
    sat = [i['value'] for i in list_sat] if sat == 'ALL' else [sat]
    if graph_type is None or site is None or sat is None or xaxis is None or yaxis is None:
        return get_empty_graph("Make sure a value for all the Dropdown Menu is selected")
    else:
        site_, sat_, x_, y_ = db.get_series('Measurements', None, site, sat, xaxis, yaxis)
        trace = []
        for i in range(len(x_)):
            # print(i, ' out of ', len(x_))
            trace.append(generate_trace(graph_type, x_[i][exclude:], y_[i][exclude:], f'{site_[i]} {sat_[i]}'))
        fig = go.Figure(data=trace)
        fig.update_layout(xaxis=dict(rangeslider=dict(visible=True)), yaxis=dict(fixedrange=False, tickformat='.3f'), height=800)
        fig.layout.autosize = True
        if graph_type == "Polar":
            if (yaxis == "Elevation"):
                fig.update_polars(radialaxis_range=[90,0])
            if (xaxis == "Azimuth"):
                fig.update_polars(angularaxis_direction="clockwise")

    return fig




def layout():
    if db.MONGO_CL == None:
        return html.Div([html.P("First you will need to select a DB in the Db Info menu")])
    else:
        return html.Div(
            [
                dropdown_type,
                dropdown_site(db.DB_SITE),
                dropdown_sat(db.DB_SAT),
                dropdown_key_x(),
                dropdown_key_y(),
                update_button,
                exclude_start(),
                dcc.Graph(
                    id='plot1',
                    figure=get_empty_graph("select information first")
                )
            ]
        )