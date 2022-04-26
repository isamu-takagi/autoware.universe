# autoware_ad_api_msgs

## InterfaceVersion

This API provides the interface version of the set of AD APIs.
It follows [Semantic Versioning][semver] in order to provide an intuitive understanding of the changes between versions.

### Use cases

Considering the product life cycle, there will be multiple vehicles that use different versions of the AD API due to changes in requirements or some improvements.
For example, a vehicle uses `v1` for stability and another vehicle uses `v2` for more functionality.

In that situation, the AD API users such as developers of a Web service have to switch the application behavior based on the version that each vehicle uses.

## ResponseStatus

This status is commonly used in software to unify error handling. The field `summary` is the overall result and processing is based on it.
The field `details` is used to pass the result of the interface used internally. This is for analysis and is mainly used by developers.

Below is a sample response status. The user knows that an parameter error has occurred in `sample/module2`, so contacting the developer with this information will facilitate the analysis.

```yaml
summary:
  code: ERROR
  component: sample
  message: ...
  description: ...
details:
  - code: SUCCESS
    component: sample/module1
    message: ...
    description: ...
  - code: ERROR
    component: sample/module2
    message: unknown parameter
    description: ...
```

## ResponseStatusDetail

This is the content of ResponseStatus. The `code` is the information for the program to process the result, and the rest are the text for the user.
The `message` is an error summary and the application first displays this to the user.
The `description` is an error detail such as solution tips and URL to the documentation. It is displayed when requested by the user.
The `component`indicates where the error occurred. which is mainly used when contacting the developer.

<!-- link -->

[semver]: https://semver.org/
