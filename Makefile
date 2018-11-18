clean:
	find . -name '*.pyc' -delete
	rm -rf __pycache__
	rm -rf .pytest_cache