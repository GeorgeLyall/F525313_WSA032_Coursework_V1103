# Bot Performance Analysis
### A student guide to reading and analysing ecosystem data

---

## Introduction

When the ecosystem runs, every bot, deliverable and station is continuously tracked by the ecosystem engine. As the simulation advances hour by hour — driven by `es.update()` — the engine recalculates and records key values for every object: how far each bot has moved, how much energy it has consumed, how many deliveries it has completed, and so on.

All of this live data is held in a central store called the **registry**. The registry is updated automatically — your operation code does not need to write to it directly. Your job as a student is to read from it at the end of a run (or during a run) to evaluate and compare the performance of your bots.

The metrics are accumulated in two main places inside the ecosystem engine:

- **`es.update()`** — called once per simulation hour. Updates position, speed, energy consumed, state of charge, active hours, age, and damage for every bot.
- **`es.contract()`** — called when a bot agrees, collects, or completes a delivery. Updates `weight_delivered` and `units_delivered`.

---

## The Registry

### Structure

The registry is a Python **dictionary of dictionaries**, stored internally as `es._registry`. The outer key is the Python object ID (`id(thing)`) of each registered object. The inner dictionary is a snapshot of every attribute of that object.

```python
{
    id(bot1): { 'name': 'rob1', 'kind': 'Robot', 'soc': 612.0, 'distance': 1450.2, ... },
    id(bot2): { 'name': 'rob2', 'kind': 'Robot', 'soc': 740.0, 'distance': 1102.7, ... },
    id(pizza1): { 'name': 'pizza1', 'kind': 'Pizza', 'status': 'delivered', ... },
    ...
}
```

Each inner dictionary is a **register** — a complete record of that object's attributes at the last update.

---

### Attribute Reference

The table below lists the key attributes in the registry for bot objects, grouped by how they behave during a run.

#### Fixed at creation — do not change during operation

| Attribute | Example | Description |
|---|---|---|
| `kind` | `'Robot'` | Specific type: Robot, Droid, or Drone |
| `kind_class` | `'Bot'` | Parent class: Bot, Deliverable, or Station |
| `name` | `'rob1'` | Unique name assigned at creation |
| `volitant` | `True` / `False` | Whether the bot flies (Drone = True) |
| `max_speed` | `1`, `2`, `3` | Top speed the bot is capable of |
| `max_payload` | `125.0` | Maximum cargo weight the bot can carry |
| `max_soc` | `750.0` | Battery capacity (maximum state of charge) |
| `weight` | `250.0` | Bot's own physical weight |
| `resources` | `[70,4,5,1,20,1]` | Material composition percentages (not used) |
| `service_freq` | `720` | Hours between required services |
| `size` | `500` | Display size (also determines weight and capacity) |

#### Accumulate throughout operation — grow monotonically

| Attribute | Description |
|---|---|
| `distance` | Total distance travelled (units) |
| `energy` | Total energy consumed (kWh equivalent) |
| `active` | Hours the bot has been in `'on'` status |
| `age` | Total hours since creation |
| `weight_delivered` | Cumulative weight of all pizzas delivered |
| `units_delivered` | Cumulative count of pizzas delivered |
| `damage` | Damage points accrued (illegal moves, running out of charge) |
| `serviced` | Age of bot at its last service |

#### Variable — change state during operation

| Attribute | Possible values | Description |
|---|---|---|
| `status` | `'off'`, `'on'`, `'broken'` | Operational state of the bot |
| `activity` | `'idle'`, `'delivering'`, `'charging'`, `'moving'`, ... | Current task |
| `soc` | `0.0` → `max_soc` | Current battery charge level |
| `speed` | `0` → `max_speed` | Speed during last update |
| `coordinates` | `[x, y]` | Current position on the arena |
| `destination` | `[x, y]` or `None` | Current movement target |
| `payload` | `0` → `max_payload` | Current cargo weight being carried |
| `cargo` | list of deliverable objects | Items currently in the cargo hold |
| `station` | station object or `None` | Station the bot is heading to / occupying |
| `alpha` | `0.0` → `1.0` | Display transparency (fades as battery depletes) |
| `color` | string | Display colour (`'black'` when broken) |

---

## Using the Registry

### Direct dictionary access

The raw registry is a standard Python dictionary. You can iterate over it, filter it, or inspect individual entries using normal dictionary methods.

```python
# Get all registry entries as a list of dictionaries
all_registers = list(es.registry().values())

# Get the register for a specific known bot object
bot_register = es.registry()[id(bot)]
print(bot_register['weight_delivered'])

# Iterate over all entries and print name and distance
for register in es.registry().values():
    print(register['name'], register['distance'])
```

### Filtering with ecosystem methods

The ecosystem provides methods that filter the registry for you and return the results as a list of register dictionaries:

```python
# All entries (unfiltered)
es.registry()

# Only bots
es.registry(kind_class='Bot')

# Only drones
es.registry(kind='Drone')

# Only broken bots
es.registry(kind_class='Bot', status='broken')
```

You can also work with the live object instances (rather than their registry snapshots) using the `things()` family of methods:

```python
# Iterate over live bot objects
for bot in es.bots():
    print(bot.name, bot.soc, bot.status)

# Filter live objects — only bots that are broken
broken = es.things(kind_class='Bot', status='broken')

# Filter live objects — only on-status drones
active_drones = es.things(kind='Drone', status='on')
```

### Tabulating registry data

The ecosystem provides a `tabulate()` method that prints a formatted table. Pass attribute names as positional arguments to select columns. Use keyword arguments to filter rows, exactly as you would with `es.registry()`.

```python
# All attributes for all bots
es.tabulate(kind_class='Bot')

# Selected columns for bots only
es.tabulate('name', 'kind', 'status', 'soc', 'active',
            'weight_delivered', 'units_delivered', 'distance', 'energy',
            kind_class='Bot')

# Only broken bots
es.tabulate('name', 'kind', 'status', 'damage',
            kind_class='Bot', status='broken')
```
Typical output of the tabulate method
```
Tabulated Data
name   kind    status   soc   active   weight_delivered   units_delivered   distance   energy   
________________________________________________________________________________________________
rob1   Robot   on       748.0 2016     388                26                1945.89    3892     
rob2   Robot   on       714.0 2016     393                27                1941.97    3886     
rob3   Robot   on       724.0 2016     497                32                1933.35    3868     
drd1   Droid   on       444.0 2016     843                57                3748.24    3748     
drd2   Droid   on       376.0 2016     881                57                3749.62    3750     
drd3   Droid   on       328.0 2016     842                53                3766.01    3766     
drn1   Drone   on       82.0  2016     1205               77                5379.22    3586     
drn2   Drone   broken   0.0   634      369                24                1698.03    1132     
drn3   Drone   on       16.0  2016     1091               72                5421.14    3614     

```



---

## Analysis

Once you can access the registry you can use standard Python and matplotlib to produce charts. The example below shows the key idea: extract two lists from the registry — one for labels, one for values — and pass them to a bar chart.

### Example: pizzas delivered per bot

```python
import matplotlib.pyplot as plt

bot_registers   = list(es.registry(kind_class='Bot').values())
names           = [r['name']            for r in bot_registers]
units_delivered = [r['units_delivered'] for r in bot_registers]

fig, ax = plt.subplots()
ax.bar(names, units_delivered)
ax.set_xlabel('Bot')
ax.set_ylabel('Pizzas delivered')
ax.set_title('Pizzas delivered per bot')
plt.show()
```

The same pattern works for any attribute in the registry — just change the key. As an extension, consider how you might use subplots to compare bot kinds separately, and calculate the mean and standard deviation of each kind's performance. Your instructor will demonstrate an example of this approach in class.

### Flagging broken bots

```python
for r in es.registry(kind_class='Bot').values():
    marker = '*** BROKEN ***' if r['status'] == 'broken' else ''
    print(f"{r['name']:<8} {r['kind']:<6} {r['status']:<8} soc:{r['soc']:.1f}  {marker}")
```

---

## Comparing Runs

A common analysis task is to vary a single parameter — such as charge threshold — and measure how the ecosystem performs under each setting. The cleanest approach is to store each completed **ecosystem object** in a results dictionary. Because the entire registry is held inside the ecosystem object, nothing is lost — you can query any metric from any run after all runs have completed.

### Setting up a parameter sweep

```python
from robots.ecosystem.factory import ecofactory
import matplotlib.pyplot as plt
import numpy as np

results = {}

for threshold in range(5, 31, 5):          # 5%, 10%, 15%, 20%, 25%, 30%

    es = ecofactory(robots=3, droids=3, drones=3, chargers=[55, 20], pizzas=9)

    charger           = es.chargers()[0]
    es.duration       = "12 week"
    es.messages_on    = False
    es.display(show=0)                      # show=0 disables the display for faster runs
    home              = [40, 20, 0]
    charge_threshold  = threshold / 100     # convert percentage to fraction

    while es.active:
        for bot in es.bots():
            if bot.soc / bot.max_soc < charge_threshold and bot.station is None:
                bot.charge(charger)
            if bot.activity == 'idle':
                for pizza in es.deliverables():
                    if pizza.status == 'ready':
                        bot.deliver(pizza)
                        break
                if not bot.destination and bot.coordinates != home:
                    bot.target_destination = home
            if bot.target_destination:
                bot.move()
        es.update()

    results[threshold] = es                 # store the whole ecosystem — all data preserved

print("All runs complete.")
```

### Analysing the results

Once all runs are stored you can extract any metric from any run. For example, total weight delivered across all bots for each threshold:

```python
for threshold, es in results.items():
    total_weight = sum(r['weight_delivered']
                       for r in es.registry(kind_class='Bot').values())
    broken_count = sum(1 for r in es.registry(kind_class='Bot').values()
                       if r['status'] == 'broken')
    print(f"Threshold {threshold:>3}%  total delivered: {total_weight:>6}  broken bots: {broken_count}")
```

### Charting the comparison

```python
thresholds    = list(results.keys())
total_weights = [
    sum(r['weight_delivered'] for r in es.registry(kind_class='Bot').values())
    for es in results.values()
]
broken_counts = [
    sum(1 for r in es.registry(kind_class='Bot').values() if r['status'] == 'broken')
    for es in results.values()
]

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))

ax1.bar([str(t) + '%' for t in thresholds], total_weights, color='steelblue')
ax1.set_xlabel('Charge threshold')
ax1.set_ylabel('Total weight delivered')
ax1.set_title('Delivery performance vs charge threshold')

ax2.bar([str(t) + '%' for t in thresholds], broken_counts, color='tomato')
ax2.set_xlabel('Charge threshold')
ax2.set_ylabel('Number of broken bots')
ax2.set_title('Bot survival vs charge threshold')

plt.tight_layout()
plt.show()
```

You can extend this pattern to any parameter: number of bots, charger location, bot size, and so on. The key principle is always the same — **one ecosystem per configuration, stored in a dictionary keyed by the parameter value**. After all runs complete, the full registry of every run is available for analysis.

---

## Stretch: Pizza Delivery Analysis

> *This section is left as an open exercise for students who want to go further.*

Everything covered so far measures performance from the **bot's perspective** — distance, energy, deliveries made. But you can also look at the ecosystem from the **pizza's perspective**: how long did each delivery take, which bots were fastest, and were there pizzas that never got delivered at all?

When a pizza completes its lifecycle it is quietly moved out of the live registry into a separate archive inside the ecosystem: `es._delivered`. This archive is a dictionary with the same structure as the main registry — each entry is a register snapshot of a delivered pizza, including its `start` hour, `end` hour, `weight`, and the name of the bot that contracted it.

```python
# Peek at the delivered pizza archive
for register in es._delivered.values():
    print(register)
```

From this data you could calculate delivery time (`end - start`) for each pizza, plot a histogram of delivery times, or find which bot had the shortest average delivery time. You could also cross-reference with the live registry to identify any pizzas that were never contracted — still sitting as `'ready'` at the end of the run.

How you structure that analysis, and what conclusions you draw, is up to you.
