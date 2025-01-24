## Class to store and display custom error messages
class ODriveError(Exception):
    def __init__(self, error_struct):
        """
        error_struct is a dictionary containing details about the error.
        Example:
            {
              "message": "ODrive error occurred",
              "time": <timestamp>,
              "details": "<any additional info>"
            }
        """
        super().__init__(error_struct["message"])
        self.error_struct = error_struct