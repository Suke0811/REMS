import plotly.graph_objects as go
import pandas as pd

# Maybe you needed to display plot in jupyter notebook
import plotly.offline as pyo

# Load exmples data
dates = ["2019-12-03", "2019-12-04", "2019-12-05", "2019-12-06",
         "2019-12-07", "2019-12-08", "2019-12-09"]
value_gold = [1477.60, 1474.45, 1448.40, 1447.40, 1444.40, 1449.40, 1441.40]
value_bitcoin = [1577.60, 1564.45, 1568.40, 1537.40, 1584.40, 1529.40, 1571.40]
df = pd.DataFrame(list(zip(dates, value_gold, value_bitcoin)),
                  columns=['date', 'value_gold', 'value_bitcoin'])

# Base plot
fig = go.Figure(
    layout=go.Layout(
        updatemenus=[dict(type="buttons", direction="right", x=0.9, y=1.16), ],
        xaxis=dict(range=["2019-12-02", "2019-12-10"],
                   autorange=False, tickwidth=2,
                   title_text="Time"),
        yaxis=dict(range=[1400, 1600],
                   autorange=False,
                   title_text="Price"),
        title="Gold - Bitcoin prices evolution",
    ))

# Add traces
init = 1

fig.add_trace(
    go.Scatter(x=df.date[:init],
               y=df.value_gold[:init],
               name="Gold",
               visible=True,
               line=dict(color="#33CFA5", dash="dash")))

fig.add_trace(
    go.Scatter(x=df.date[:init],
               y=df.value_bitcoin[:init],
               name="Bitcoin",
               visible=True,
               line=dict(color="#bf00ff", dash="dash")))

# Animation
fig.update(frames=[
    go.Frame(
        data=[
            go.Scatter(x=df.date[:k], y=df.value_gold[:k]),
            go.Scatter(x=df.date[:k], y=df.value_bitcoin[:k])]
    )
    for k in range(init, len(df)+1)])

# Extra Formatting
fig.update_xaxes(ticks="outside", tickwidth=2, tickcolor='white', ticklen=10)
fig.update_yaxes(ticks="outside", tickwidth=2, tickcolor='white', ticklen=1)
fig.update_layout(yaxis_tickformat=',')
fig.update_layout(legend=dict(x=0, y=1.1), legend_orientation="h")

# Buttons
fig.update_layout(
    updatemenus=[
        dict(
            buttons=list([
                dict(label="Play",
                        method="animate",
                    args=[None, {"frame": {"duration": 1000}}]),
                dict(label="Gold",
                    method="update",
                    args=[{"visible": [False, True]},
                          {"showlegend": True}]),
                dict(label="Bitcoin",
                    method="update",
                    args=[{"visible": [True, False]},
                          {"showlegend": True}]),
                dict(label="All",
                    method="update",
                    args=[{"visible": [True, True, True]},
                          {"showlegend": True}]),
            ]))])

#fig.show()
pyo.plot(fig, auto_play=True)

fig.add_trace(
    go.Scatter(x=[],
               y=df.value_bitcoin[:init],
               name="Bitcoin",
               visible=True,
               line=dict(color="#bf00ff", dash="dash")))
