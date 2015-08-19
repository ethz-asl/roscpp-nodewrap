time[2] stamps_of_last_events # Timestamps of the last events

uint32 num_samples            # Number of non-rolling samples
float32 min                   # Minimum latency in [s]
float32 max                   # Maximum latency in [s]

uint32 num_rolling_samples    # Number of rolling samples
float32 rolling_mean          # Rolling mean latency in [s]
float32 rolling_variance      # Rolling latency variance in [s^2]
