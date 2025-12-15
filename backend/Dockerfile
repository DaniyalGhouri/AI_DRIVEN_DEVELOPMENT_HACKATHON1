# Use a lightweight Python image as a base
FROM python:3.10-slim-buster

# Set the working directory in the container
WORKDIR /app

# Copy only the dependency files first to leverage Docker cache
COPY ./backend/Pipfile ./backend/Pipfile.lock ./

# Install dependencies
RUN pip install pipenv && pipenv install --system --deploy --ignore-pipfile

# Copy the rest of the application code
COPY ./backend /app

# Expose the port FastAPI will run on
EXPOSE 8000

# Command to run the FastAPI application
CMD ["uvicorn", "backend.main:app", "--host", "0.0.0.0", "--port", "8000"]
